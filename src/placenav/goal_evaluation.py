"""Read-only goal scores shared by selection and diagnostics, not a pose sensor."""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class GoalEvaluationConfig:
    position_tolerance: float = 0.5
    yaw_tolerance: float = math.radians(20)
    neighborhood_radius: float = 2.0

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in (
                self.position_tolerance, self.yaw_tolerance, self.neighborhood_radius)):
            raise ValueError('Goal evaluation tolerances must be finite and positive')
        if self.yaw_tolerance > math.pi:
            raise ValueError('Goal evaluation yaw tolerance must not exceed pi')
        if self.neighborhood_radius < self.position_tolerance:
            raise ValueError('Goal evaluation neighborhood must contain the arrival interval')


def node_position_score(motion, node_idx, tolerance):
    """Global mass both owned by the target node and near its recorded position.

    Ownership is the existing VPR observation partition. Integrate sub-cell
    position variance without renormalizing or treating yaw as a measurement.
    """
    if motion.probability is None or not 0 <= node_idx < len(motion.node_s):
        raise ValueError('Node position score requires an initialized valid goal')
    position = motion.node_s[node_idx]
    near = motion.interval_weights(position - tolerance, position + tolerance)
    return float(motion.probability @ (near * (motion.owner == node_idx)))


def evaluate_goal(motion, node_idx, config):
    """Sum global mass; never condition on a selected neighborhood or change it.

    Positions are route progress, not 2D distance. Heading is one conditional
    estimate per cell, initialized/reanchored to the map heading, not a measured
    absolute alignment or an orientation distribution. These are scores under
    those assumptions, not calibrated physical arrival probabilities.
    """
    if motion.probability is None:
        raise ValueError('Goal evaluation requires an initialized posterior')
    if not 0 <= node_idx < len(motion.node_s):
        raise ValueError('Invalid goal evaluation node')
    probability = motion.probability
    goal_s, goal_yaw = motion.node_s[node_idx], motion.node_yaw[node_idx]
    heading = motion.yaw + motion.heading_offset
    angle = np.arctan2(np.sin(heading - goal_yaw), np.cos(heading - goal_yaw))
    near = motion.interval_weights(goal_s - config.position_tolerance, goal_s + config.position_tolerance)
    neighborhood = motion.interval_weights(goal_s - config.neighborhood_radius, goal_s + config.neighborhood_radius)
    aligned = np.abs(angle) <= config.yaw_tolerance
    passed = motion.interval_weights(goal_s, goal_s + config.neighborhood_radius)
    mode = int(np.argmax(probability))
    quantiles = np.searchsorted(np.cumsum(probability), [.1, .5, .9])
    quantiles = motion.grid_s[np.minimum(quantiles, len(probability) - 1)]
    return dict(
        goal_s=float(goal_s), goal_yaw=float(goal_yaw),
        map_s=float(motion.grid_s[mode]),
        map_yaw=float(np.arctan2(np.sin(heading[mode]), np.cos(heading[mode]))),
        map_yaw_error=float(angle[mode]),
        position_score=float(probability @ near),
        arrival_score=float(probability @ (near * aligned)),
        neighborhood_mass=float(probability @ neighborhood),
        local_passage_score=float(probability @ passed),
        local_passage_heading_score=float(probability @ (passed * aligned)),
        q10_s=float(quantiles[0]), q50_s=float(quantiles[1]), q90_s=float(quantiles[2]),
        boundary_clipped_mass=float(motion.last_boundary_clipped_mass),
        endpoint_mass=float(probability[1] + (probability[-2] if len(probability) > 3 else 0)),
        outside_mass=float(np.clip(1 - probability @ motion.interval_weights(0, motion.node_s[-1]), 0, 1)),
        zero_node_fraction=float(np.mean(motion.node_belief() == 0)),
        heading_reference_assumed=True,
    )
