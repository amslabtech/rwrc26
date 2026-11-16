"""One conditional planar pose for diagnostics; never correct the VPR filter.

The first node is an assumed anchor, not a geometric visual measurement.
Alternative global hypotheses remain in the existing topological posterior.
"""
from dataclasses import dataclass
import math

import numpy as np

from placenav.topomap_odometry import relative_pose


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


@dataclass(frozen=True)
class LocalPoseConfig:
    lateral_std: float = .5
    yaw_std: float = math.radians(20)
    translation_noise: float = .1  # meters / sqrt(meter travelled)
    yaw_noise_per_meter: float = .05  # radians / sqrt(meter travelled)
    yaw_noise_per_radian: float = .05  # radians / sqrt(radian turned)

    def __post_init__(self):
        if any(not math.isfinite(v) or v < 0 for v in vars(self).values()):
            raise ValueError('Local pose uncertainties must be finite and nonnegative')


class LocalPoseTracker:
    """Propagate a single assumed map-to-live alignment, at image timestamps.

    Ordinary goal switches and inconsistent confirmed recovery can reanchor XY.
    Heading continues to follow odometry. Resets can be disabled independently.
    A discontinuity invalidates it until restart; corrections cannot revive it.
    """
    def __init__(self, motion, config=None):
        self.motion = motion
        self.config = config or LocalPoseConfig()
        self.mean = None
        self.covariance = None
        self.last_odometry = None
        self.epoch = None
        self.anchor_node = None
        self.anchor_stamp_ns = None
        self.anchor_probability = None
        self.switch_count = 0
        self.arrival_count = 0
        self.recovery_count = 0
        self.continuous = True
        self.status = 'UNINITIALIZED'

    def map_positions(self, progress):
        """Interpolate route XY; extend endpoint tangents instead of clipping."""
        motion = self.motion
        progress = np.atleast_1d(progress)
        xy = np.column_stack([np.interp(progress, motion.node_s, motion.node_xy[:, k])
                              for k in (0, 1)])
        for mask, endpoint, tangent in (
                (progress < 0, 0, motion._segment_directions[0]),
                (progress > motion.node_s[-1], -1, motion._segment_directions[-1])):
            xy[mask] += (progress[mask] - motion.node_s[endpoint])[:, None] * tangent
        return xy

    def _initialize(self, closest_node, odometry, epoch):
        motion, config = self.motion, self.config
        mask = (motion.owner == closest_node) & (motion.probability > 0)
        weights = motion.probability[mask]
        self.anchor_probability = float(weights.sum())
        if self.anchor_probability <= 0:
            raise ValueError('Local pose anchor has no posterior support')
        weights = weights / self.anchor_probability
        xy = self.map_positions(motion.grid_s[mask])
        heading = (motion.yaw + motion.heading_offset)[mask]
        yaw = math.atan2(float(weights @ np.sin(heading)), float(weights @ np.cos(heading)))
        self.mean = np.r_[weights @ xy, yaw]
        residual = np.column_stack([xy - self.mean[:2], wrap(heading - yaw)])
        covariance = (residual.T * weights) @ residual
        # Include within-bin progress variance and ignorance of lateral offset.
        directions = np.column_stack([motion.tangent[mask], np.zeros(len(weights))])
        covariance += (directions.T * (weights * motion.position_variance[mask])) @ directions
        normals = np.column_stack([-motion.tangent[mask, 1], motion.tangent[mask, 0], np.zeros(len(weights))])
        covariance += config.lateral_std**2 * (normals.T * weights) @ normals
        covariance[2, 2] += config.yaw_std**2
        self.covariance = covariance
        self.last_odometry = odometry
        self.epoch = epoch
        self.anchor_node = int(closest_node)
        self.anchor_stamp_ns = odometry.stamp_ns
        self.continuous = True
        self.status = 'TRACKING_ASSUMED_ANCHOR'

    def invalidate(self, reason):
        self.continuous = False
        self.status = reason

    def update(self, closest_node, odometry, epoch):
        if self.mean is None:
            self._initialize(closest_node, odometry, epoch)
            return
        if not self.continuous:
            return
        previous = self.last_odometry
        if epoch != self.epoch or (previous.frame_id, previous.child_frame_id) != (
                odometry.frame_id, odometry.child_frame_id):
            self.invalidate('ODOMETRY_DISCONTINUITY')
            return
        if odometry == previous:
            return
        if odometry.stamp_ns <= previous.stamp_ns:
            self.invalidate('ODOMETRY_TIME_DISCONTINUITY')
            return
        dx, dy, dyaw = relative_pose(previous.pose, odometry.pose)
        yaw = self.mean[2]
        c, s = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
        delta = rotation @ np.array([dx, dy, dyaw])
        jacobian = np.eye(3)
        jacobian[0, 2], jacobian[1, 2] = -delta[1], delta[0]
        distance = math.hypot(dx, dy)
        config = self.config
        variance_xy = config.translation_noise**2 * distance
        variance_yaw = config.yaw_noise_per_meter**2 * distance + config.yaw_noise_per_radian**2 * abs(dyaw)
        noise = np.diag([variance_xy, variance_xy, variance_yaw])
        # Integrate yaw process noise along a straight increment instead of
        # adding all of it at the end. Preserve position/yaw cross covariance.
        lever = np.array([-dy, dx])
        noise[:2, :2] += variance_yaw * np.outer(lever, lever) / 3
        noise[:2, 2] = noise[2, :2] = variance_yaw * lever / 2
        covariance = jacobian @ self.covariance @ jacobian.T + rotation @ noise @ rotation.T
        self.covariance = (covariance + covariance.T) / 2
        self.mean += delta
        self.mean[2] = wrap(self.mean[2])
        self.last_odometry = odometry
        self.status = ('TRACKING_POSITION_ANCHOR' if self.switch_count or self.recovery_count else
                       'TRACKING_ASSUMED_ANCHOR')

    def _set_position_anchor(self, node, stamp, position, covariance, status):
        self.mean[:2] = position
        self.covariance[:2, :2] = covariance
        # A new assumed XY anchor does not measure or correct heading.
        self.covariance[:2, 2] = self.covariance[2, :2] = 0.
        self.anchor_node = int(node)
        self.anchor_stamp_ns = stamp
        self.anchor_probability = float(self.motion.node_belief()[node])
        self.status = status

    def correct_subgoal_switch(self, selector, previous_goal=None, closest_node=None, enabled=True):
        """Apply the same position-only anchor at a mode's ordinary goal switch.

        Passage/arrival anchor the evaluated old goal. Nearest anchors its
        current-node estimate, including backward changes and lookahead > 1.
        None of these events independently measures geometric arrival.
        """
        if (not enabled or self.mean is None or not self.continuous or
                selector.motion is not self.motion or selector.reached):
            return False
        if selector.mode == 'nearest':
            if previous_goal is None or previous_goal == selector.subgoal or selector.status == 'INITIAL':
                return False
            node = closest_node
        elif selector.mode in ('arrival', 'passage'):
            if selector.status != 'ADVANCED_' + selector.mode.upper():
                return False
            node = selector.decision_goal_idx
            if selector.subgoal != node + 1 or (previous_goal is not None and previous_goal != node):
                return False
        else:
            return False
        stamp = selector.last_stamp
        if (node is None or not 0 <= node < len(self.motion.node_s) or
                stamp != self.last_odometry.stamp_ns or stamp <= self.anchor_stamp_ns):
            return False
        # An anchor is assumed, with the same uncertainty floor in every mode.
        std = max(self.config.lateral_std, selector.evaluation_config.position_tolerance)
        self._set_position_anchor(node, stamp, self.motion.node_xy[node],
                                  np.eye(2) * std**2, selector.mode.upper() + '_POSITION_RESET')
        self.switch_count += 1
        if selector.mode == 'arrival':
            self.arrival_count += 1
        return True

    def correct_arrival(self, selector):
        """Arrival-only entry point retained for existing bag audit scripts."""
        if selector.mode != 'arrival' or selector.status != 'ADVANCED_ARRIVAL':
            return False
        return self.correct_subgoal_switch(selector)

    def correct_recovery(self, selector, closest_node, enabled=True):
        """Repair a stale XY reference only after coherent goal recovery.

        Use the selected metric neighborhood, not a mean over remote modes.
        This is an assumed route position, not an independent visual pose.
        Small disagreements retain lateral avoidance offsets and the anchor.
        """
        if (not enabled or self.mean is None or not self.continuous or selector.motion is not self.motion or
                selector.mode not in ('arrival', 'passage') or
                selector.status not in ('RECOVERED_FORWARD', 'RECOVERED_BACKWARD')):
            return False
        stamp = selector.last_stamp
        if (not 0 <= closest_node < len(self.motion.node_s) or
                selector.subgoal != min(closest_node + selector.lookahead, selector.final_node) or
                stamp != self.last_odometry.stamp_ns or stamp <= self.anchor_stamp_ns):
            return False
        motion = self.motion
        radius = selector.evaluation_config.neighborhood_radius
        position = motion.node_s[closest_node]
        weights = motion.probability * motion.interval_weights(position - radius, position + radius)
        # A distant cell's Gaussian tail is not a local XY hypothesis. Its
        # untruncated mean would otherwise pull the reference toward that mode.
        weights[np.abs(motion.grid_s - position) > radius] = 0.
        mass = float(weights.sum())
        if mass < selector.recovery_threshold:
            return False
        mask = weights > 0
        weights = weights[mask] / mass
        xy = self.map_positions(motion.grid_s[mask])
        mean = weights @ xy
        if np.linalg.norm(mean - self.mean[:2]) <= radius:
            return False
        residual = xy - mean
        covariance = (residual.T * weights) @ residual
        directions = motion.tangent[mask]
        covariance += (directions.T * (weights * motion.position_variance[mask])) @ directions
        # Retain posterior spread and a floor for unobserved lateral position.
        std = max(self.config.lateral_std, selector.evaluation_config.position_tolerance)
        covariance += np.eye(2) * std**2
        self._set_position_anchor(closest_node, stamp, mean, covariance, 'RECOVERY_POSITION_RESET')
        self.recovery_count += 1
        return True

    def goal(self, node_idx):
        """Goal pose/covariance in the current body frame, conditional on anchor.

        The recorded map pose is treated as fixed. Its recording errors and
        the probability of choosing a wrong global anchor are not covariance.
        """
        if self.mean is None:
            raise ValueError('Local pose is not initialized')
        target = np.r_[self.motion.node_xy[node_idx], self.motion.node_yaw[node_idx]]
        relative = np.array(relative_pose(self.mean, target))
        c, s = math.cos(self.mean[2]), math.sin(self.mean[2])
        jacobian = np.array([[-c, -s, relative[1]],
                             [s, -c, -relative[0]], [0., 0., -1.]])
        covariance = jacobian @ self.covariance @ jacobian.T
        return relative, (covariance + covariance.T) / 2

    def belief_disagreement(self):
        """Distance/angle to the 1D MAP cell; diagnostic, not another likelihood."""
        index = int(np.argmax(self.motion.probability))
        xy = self.map_positions([self.motion.grid_s[index]])[0]
        yaw = self.motion.yaw[index] + self.motion.heading_offset[index]
        return (float(np.linalg.norm(xy - self.mean[:2])),
                float(wrap(yaw - self.mean[2])))
