"""Image heading alignment at subgoal arrivals, independent of ROS and models.

OmniVLA-edge follows a goal image only while the required rotation is small
(about +-25-30 deg), and at larger ones it can even turn the wrong way. Topomaps
record turns on the spot as several nodes at nearly the same position. With
this gate, a positional arrival at subgoal k advances the subgoal only once the
robot faces node k's recorded heading. The heading difference dpsi (live -
node, CCW +) is measured between the live image and node k's image (e.g.
SuperPoint+LightGlue, rotation-only); above `start_deg` the robot rotates in
place until it is below `done_deg`. The rotation happens where the map run
turned, so the robot does not cut the corner, and the next subgoal is handed to
the waypoint model only within its working range.

Every successful image measurement becomes the *anchor* (node, dpsi, odometry
yaw, time). While the robot approaches its subgoal (`approach_distance`), the
subgoal is also measured every `approach_interval` s, because at the arrival
itself the camera may face a blank wall (indoor corners). When the image
cannot be measured at an arrival or during a rotation, dpsi of node k is
predicted from the anchor a:

    dpsi_k = dpsi_a + (yaw_now - yaw_a) - (taught_yaw_k - taught_yaw_a)

The odometry yaw spans only the seconds since the anchor and the taught yaw
difference only the few nodes between them (anchors expire after
`anchor_max_age` s or `anchor_max_nodes` nodes); no accumulated odometry
heading is used. Without any usable anchor the gate holds for
`fallback_after` frames and then assumes the robot faces node k-1's heading
(taught step). A rotation that exceeds `timeout` s lets the subgoal advance.

Per localization cycle: new_cycle(measure, odom_yaw, stamp, subgoal, remaining)
-> the subgoal selector calls gate(node) on a positional arrival ->
finish_cycle(subgoal) (which may take the approach measurement).
"""
from dataclasses import dataclass
import math

import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class HeadingAlignConfig:
    start_deg: float = 20.         # |dpsi| above which a rotation starts
    done_deg: float = 13.          # |dpsi| that ends it; keep above TEB yaw_goal_tolerance (0.2 rad = 11.5 deg)
    timeout: float = 20.           # seconds of rotation before the subgoal advances anyway
    fallback_after: int = 5        # frames without measurement or anchor before the taught-step fallback
    approach_distance: float = 2.  # route metres before the subgoal where it is also measured (0: off)
    approach_interval: float = 0.5  # seconds between approach measurements
    anchor_max_age: float = 15.    # seconds an image measurement may carry a prediction
    anchor_max_nodes: int = 8      # nodes between the anchor and the predicted node

    def __post_init__(self):
        if not 0 < self.done_deg <= self.start_deg < 180:
            raise ValueError('Heading alignment needs 0 < done_deg <= start_deg < 180')
        if not (math.isfinite(self.timeout) and self.timeout > 0) or self.fallback_after < 1:
            raise ValueError('Invalid heading alignment timeout or fallback count')
        if (not math.isfinite(self.approach_distance) or self.approach_distance < 0 or
                not math.isfinite(self.approach_interval) or self.approach_interval < 0 or
                not math.isfinite(self.anchor_max_age) or self.anchor_max_age < 0 or self.anchor_max_nodes < 0):
            raise ValueError('Invalid heading alignment approach or anchor limits')


@dataclass
class AlignCommand:
    active: bool               # rotate in place to target_yaw (or hold) instead of following waypoints
    status: str                # IDLE, APPROACH, ALIGNED(_ANCHOR/_TAUGHT/_ODOM), ROTATING(_ANCHOR/_ODOM),
                               # TIMEOUT, WAITING_IMAGE
    node: int = -1             # node whose heading is being matched
    target_yaw: float = math.nan  # odometry frame
    dpsi: float = math.nan     # live - node heading (rad, CCW +): measured, or predicted from the anchor
    support: int = 0           # inliers of an image measurement (0 if predicted)
    source: str = ''           # image, anchor (prediction from an earlier image) or odometry (taught step)


@dataclass
class Anchor:
    node: int
    dpsi: float
    yaw: float
    stamp: float


class HeadingAligner:
    def __init__(self, node_yaw, config=None):
        self.config = config or HeadingAlignConfig()
        self.node_yaw = np.unwrap(np.asarray(node_yaw, dtype=float))
        self.start = math.radians(self.config.start_deg)
        self.done = math.radians(self.config.done_deg)
        self.aligned_node = None
        self.anchor = None
        self._last_approach = -math.inf
        self._reset()
        self._cycle = None

    def _reset(self):
        self.node = None            # node being aligned (latched across cycles)
        self.rotating = False
        self.source = ''
        self.target_yaw = math.nan
        self.started = None
        self.failures = 0
        self.last = AlignCommand(False, 'IDLE')

    # -- per cycle --------------------------------------------------------
    def new_cycle(self, measure, odom_yaw, stamp_s, subgoal=None, remaining=None):
        """measure(node) -> HeadingEstimate or None for the current image; remaining: route m to subgoal."""
        self._cycle = {'measure': measure, 'yaw': float(odom_yaw), 'stamp': float(stamp_s), 'done': False,
                       'result': None, 'subgoal': subgoal, 'remaining': remaining}

    def gate(self, node):
        """Called on a positional arrival at subgoal `node`: True lets the subgoal advance."""
        if self.aligned_node == node:
            return True
        if self.node != node:
            self._reset()
            self.node = node
        self._step()
        return self.aligned_node == node

    def finish_cycle(self, subgoal, reached_goal=False):
        """Continue a latched rotation, or take an approach measurement; return the command."""
        if reached_goal:
            self._reset()
            self._cycle = None
            return AlignCommand(False, 'IDLE')
        if self.node is not None and self.node != subgoal:
            self._reset()  # the subgoal moved (advance or recovery)
        if self.aligned_node != subgoal:
            self.aligned_node = None
        cycle = self._cycle
        if self.node is not None and cycle is not None and not cycle['done']:
            self._step()
        elif self.node is None and cycle is not None and not cycle['done']:
            approach = self._approach(cycle, subgoal)
            if approach is not None:
                cycle['result'] = approach
        result = cycle['result'] if cycle is not None else None
        self._cycle = None
        if self.node is not None:
            return self.last
        # Report a decision taken in this cycle (ALIGNED with its dpsi, APPROACH, TIMEOUT), then IDLE.
        return result if result is not None else AlignCommand(False, 'IDLE')

    # -- internals --------------------------------------------------------
    def _approach(self, cycle, subgoal):
        config = self.config
        remaining = cycle['remaining']
        if (subgoal is None or remaining is None or config.approach_distance <= 0 or
                remaining > config.approach_distance or
                cycle['stamp'] - self._last_approach < config.approach_interval):
            return None
        cycle['done'] = True
        self._last_approach = cycle['stamp']
        estimate = cycle['measure'](subgoal)
        if estimate is None:
            return None
        self.anchor = Anchor(subgoal, float(estimate.dpsi), cycle['yaw'], cycle['stamp'])
        return AlignCommand(False, 'APPROACH', subgoal, math.nan, float(estimate.dpsi), estimate.support, 'image')

    def _predict(self, node, yaw, now):
        """dpsi of `node` from the anchor, or None when there is no recent, nearby anchor."""
        a, config = self.anchor, self.config
        if a is None or now - a.stamp > config.anchor_max_age or abs(node - a.node) > config.anchor_max_nodes:
            return None
        return wrap(a.dpsi + wrap(yaw - a.yaw) - (self.node_yaw[node] - self.node_yaw[a.node]))

    def _finish(self, status, dpsi=math.nan, support=0, source=''):
        node = self.node
        self.aligned_node = node
        self._reset()
        self.last = AlignCommand(False, status, node, math.nan, dpsi, support, source)
        return self.last

    def _step(self):
        cycle = self._cycle
        if cycle is None or cycle['done']:
            return self.last
        cycle['done'] = True
        cycle['result'] = self._decide(cycle)
        return cycle['result']

    def _decide(self, cycle):
        yaw, now, node = cycle['yaw'], cycle['stamp'], self.node
        if self.started is not None and now - self.started > self.config.timeout:
            return self._finish('TIMEOUT')
        estimate = cycle['measure'](node)
        support, source = 0, ''
        if estimate is not None:
            self.failures = 0
            dpsi, support, source = float(estimate.dpsi), estimate.support, 'image'
            self.anchor = Anchor(node, dpsi, yaw, now)
        else:
            self.failures += 1
            dpsi = self._predict(node, yaw, now)
            source = 'anchor' if dpsi is not None else ''
            if dpsi is None and self.rotating and self.source == 'odometry':
                if abs(wrap(self.target_yaw - yaw)) <= self.done:
                    return self._finish('ALIGNED_ODOM', source='odometry')
                self.last = AlignCommand(True, 'ROTATING_ODOM', node, self.target_yaw, math.nan, 0, 'odometry')
                return self.last
        if dpsi is not None:
            limit = self.done if self.rotating else self.start
            if abs(dpsi) <= limit:
                return self._finish('ALIGNED' if source == 'image' else 'ALIGNED_ANCHOR', dpsi, support, source)
            if self.started is None:
                self.started = now
            self.rotating, self.source = True, source
            self.target_yaw = wrap(yaw - dpsi)  # dpsi holds at this image's odometry yaw
            status = 'ROTATING' if source == 'image' else 'ROTATING_ANCHOR'
            self.last = AlignCommand(True, status, node, self.target_yaw, dpsi, support, source)
            return self.last
        if self.rotating:  # earlier image target, no anchor any more: keep turning to it
            self.last = AlignCommand(True, self.last.status, node, self.target_yaw, self.last.dpsi, 0, self.source)
            return self.last
        if self.failures < self.config.fallback_after:
            # Hold position and heading (do not drive past the turn) while waiting for a measurement.
            if not math.isfinite(self.target_yaw):
                self.target_yaw = yaw
            self.last = AlignCommand(True, 'WAITING_IMAGE', node, self.target_yaw, math.nan, 0, '')
            return self.last
        # Rescue without any image: assume the robot faces node k-1's heading; rotate by the taught step.
        step = float(self.node_yaw[node] - self.node_yaw[node - 1]) if node > 0 else 0.
        if abs(step) <= self.done:
            return self._finish('ALIGNED_TAUGHT', source='odometry')
        self.started = now
        self.rotating, self.source = True, 'odometry'
        self.target_yaw = wrap(yaw + step)
        self.last = AlignCommand(True, 'ROTATING_ODOM', node, self.target_yaw, math.nan, 0, 'odometry')
        return self.last
