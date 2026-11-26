"""Image heading alignment at subgoal arrivals, independent of ROS and models.

OmniVLA-edge follows a goal image only while the required rotation is small
(about +-25-30 deg), and at larger ones it can even turn the wrong way. Topomaps
record turns on the spot as several nodes at nearly the same position. With
this gate, a positional arrival at subgoal k advances the subgoal only once the
live image faces node k's recorded heading: the heading difference dpsi is
measured between the live image and node k's image (e.g. SuperPoint+LightGlue,
rotation-only), and above `start_deg` the robot rotates in place until it is
below `done_deg`. The rotation happens where the map run turned, so the robot
does not cut the corner, and the next subgoal is handed to the waypoint model
only within its working range. No accumulated odometry heading is used: the
odometry yaw only carries an image measurement over the few seconds of one
rotation.

If no image measurement succeeds (`fallback_after` consecutive frames), the
gate falls back to the taught yaw step between node k-1 and node k, assuming
the robot faces node k-1's heading, and rotates by it with odometry. A
rotation that exceeds `timeout` seconds lets the subgoal advance anyway.

Per localization cycle: new_cycle(measure, odom_yaw, stamp) -> the subgoal
selector calls gate(node) on a positional arrival -> finish_cycle(subgoal).
"""
from dataclasses import dataclass
import math

import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class HeadingAlignConfig:
    start_deg: float = 25.    # |dpsi| above which a rotation starts
    done_deg: float = 15.     # |dpsi| that ends it; keep above TEB yaw_goal_tolerance (0.2 rad)
    timeout: float = 20.      # seconds of rotation before the subgoal advances anyway
    fallback_after: int = 5   # consecutive frames without a measurement before the taught-step fallback

    def __post_init__(self):
        if not 0 < self.done_deg <= self.start_deg < 180:
            raise ValueError('Heading alignment needs 0 < done_deg <= start_deg < 180')
        if not (math.isfinite(self.timeout) and self.timeout > 0) or self.fallback_after < 1:
            raise ValueError('Invalid heading alignment timeout or fallback count')


@dataclass
class AlignCommand:
    active: bool               # rotate in place to target_yaw (or hold) instead of following waypoints
    status: str                # IDLE, ALIGNED(_ODOM/_TAUGHT), ROTATING(_ODOM), TIMEOUT, WAITING_IMAGE
    node: int = -1             # node whose heading is being matched
    target_yaw: float = math.nan  # odometry frame
    dpsi: float = math.nan     # latest image measurement, live - node heading (rad, CCW +)
    support: int = 0
    source: str = ''           # image or odometry


class HeadingAligner:
    def __init__(self, node_yaw, config=None):
        self.config = config or HeadingAlignConfig()
        self.node_yaw = np.unwrap(np.asarray(node_yaw, dtype=float))
        self.start = math.radians(self.config.start_deg)
        self.done = math.radians(self.config.done_deg)
        self.aligned_node = None
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
    def new_cycle(self, measure, odom_yaw, stamp_s):
        """measure(node) -> HeadingEstimate or None, for the current image."""
        self._cycle = {'measure': measure, 'yaw': float(odom_yaw), 'stamp': float(stamp_s), 'done': False,
                       'result': None}

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
        """Continue a latched rotation also when the arrival condition flickered; return the command."""
        if reached_goal:
            self._reset()
            return AlignCommand(False, 'IDLE')
        if self.node is not None and self.node != subgoal:
            self._reset()  # the subgoal moved (advance or recovery)
        if self.aligned_node != subgoal:
            self.aligned_node = None
        if self.node is not None and self._cycle is not None and not self._cycle['done']:
            self._step()
        result = self._cycle['result'] if self._cycle is not None else None
        self._cycle = None
        if self.node is not None:
            return self.last
        # Report a decision taken in this cycle (e.g. ALIGNED with its dpsi), then IDLE.
        return result if result is not None else AlignCommand(False, 'IDLE')

    # -- internals --------------------------------------------------------
    def _finish(self, status):
        node = self.node
        self.aligned_node = node
        self._reset()
        self.last = AlignCommand(False, status, node)
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
        if estimate is not None:
            self.failures = 0
            dpsi = float(estimate.dpsi)
            limit = self.done if self.rotating else self.start
            if abs(dpsi) <= limit:
                self._finish('ALIGNED')
                self.last.dpsi, self.last.support, self.last.source = dpsi, estimate.support, 'image'
                return self.last
            if self.started is None:
                self.started = now
            self.rotating, self.source = True, 'image'
            self.target_yaw = wrap(yaw - dpsi)  # dpsi is measured at this image's odometry yaw
            self.last = AlignCommand(True, 'ROTATING', node, self.target_yaw, dpsi, estimate.support, 'image')
            return self.last
        self.failures += 1
        if self.rotating:
            if self.source == 'odometry' and abs(wrap(self.target_yaw - yaw)) <= self.done:
                return self._finish('ALIGNED_ODOM')
            # Keep turning to the last target; a new image measurement refines it.
            self.last = AlignCommand(True, self.last.status, node, self.target_yaw, self.last.dpsi,
                                     0, self.source)
            return self.last
        if self.failures < self.config.fallback_after:
            # Hold position and heading (do not drive past the turn) while waiting for a measurement.
            if not math.isfinite(self.target_yaw):
                self.target_yaw = yaw
            self.last = AlignCommand(True, 'WAITING_IMAGE', node, self.target_yaw, math.nan, 0, '')
            return self.last
        # Rescue: assume the robot still faces node k-1's heading; rotate by the taught step.
        step = float(self.node_yaw[node] - self.node_yaw[node - 1]) if node > 0 else 0.
        if abs(step) <= self.start:
            return self._finish('ALIGNED_TAUGHT')
        self.started = now
        self.rotating, self.source = True, 'odometry'
        self.target_yaw = wrap(yaw + step)
        self.last = AlignCommand(True, 'ROTATING_ODOM', node, self.target_yaw, math.nan, 0, 'odometry')
        return self.last
