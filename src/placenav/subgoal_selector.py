"""Metric goal decisions from the posterior, independent of ROS and models."""
import math

import numpy as np

from placenav.goal_evaluation import GoalEvaluationConfig, evaluate_goal, node_position_score


class ProgressSubgoalSelector:
    def __init__(self, motion, final_node, mode='nearest', lookahead=1,
                 threshold=0.5, confirmations=2, min_local_mass=0.8,
                 final_tolerance=0.5, evaluation_config=None,
                 recovery_distance=4.0, recovery_threshold=0.8):
        # arrival_turn shares all arrival behavior except the relative-yaw gate.
        self.turn_enabled = mode == 'arrival_turn'
        self.turn_anchor = None
        if self.turn_enabled:
            mode = 'arrival'
        if mode not in ('nearest', 'passage', 'arrival'):
            raise ValueError('Unknown subgoal update mode')
        if mode != 'nearest' and lookahead != 1:
            raise ValueError('passage/arrival require lookahead=1')
        if not 0 <= final_node < len(motion.node_s):
            raise ValueError('Invalid final node')
        if any(not math.isfinite(v) or not 0 < v <= 1
               for v in (threshold, min_local_mass, recovery_threshold)):
            raise ValueError('Probability thresholds must be in (0, 1]')
        if not isinstance(confirmations, int) or confirmations < 1:
            raise ValueError('confirmations must be a positive integer')
        if not math.isfinite(final_tolerance) or final_tolerance < 0:
            raise ValueError('final_tolerance must be finite and nonnegative')
        self.evaluation_config = evaluation_config or GoalEvaluationConfig()
        if (not math.isfinite(recovery_distance) or
                recovery_distance < self.evaluation_config.neighborhood_radius):
            raise ValueError('Recovery distance must cover the metric neighborhood')
        self.motion, self.final_node = motion, final_node
        self.mode, self.lookahead = mode, lookahead
        self.threshold, self.confirmations = threshold, confirmations
        self.min_local_mass, self.final_tolerance = min_local_mass, final_tolerance
        self.recovery_distance, self.recovery_threshold = recovery_distance, recovery_threshold
        self.subgoal = None
        self.reached = False
        self.last_stamp = None
        self.count = 0
        self.recovery_count = 0
        self.recovery_candidate = None
        self.status = 'INITIAL'
        self.probabilities = {}
        self.decision_goal_idx = -1
        self.decision_score = 0.

    def clear_odometry_reference(self):
        self.turn_anchor = None
        self.reset_confirmation()

    def _set_turn_anchor(self, node):
        if self.turn_enabled and self.motion.last_odometry is not None:
            self.turn_anchor = (node, self.motion.last_odometry.pose[2])

    def _turn_satisfied(self):
        if self.turn_anchor is None:
            # First position-confirmed arrival establishes the odometry reference.
            return True
        node, start_yaw = self.turn_anchor
        delta = self.motion.node_yaw[self.subgoal] - self.motion.node_yaw[node]
        taught = math.atan2(math.sin(delta), math.cos(delta))
        if abs(taught) < math.radians(15):
            return True
        sample = self.motion.last_odometry
        if sample is None:
            return False
        delta = sample.pose[2] - start_yaw
        measured = math.atan2(math.sin(delta), math.cos(delta))
        return math.copysign(1., taught) * measured >= 0.5 * abs(taught)

    def reset_confirmation(self):
        self.count = 0
        self.recovery_count = 0
        self.recovery_candidate = None

    def update(self, closest_node, stamp_ns):
        if self.reached or stamp_ns == self.last_stamp:
            return self.subgoal, self.reached
        if self.last_stamp is not None and stamp_ns < self.last_stamp:
            self.clear_odometry_reference()
        self.last_stamp = stamp_ns
        initial = self.subgoal is None
        if initial or self.mode == 'nearest':
            candidate = min(closest_node + self.lookahead, self.final_node)
            if candidate != self.subgoal:
                self.reset_confirmation()
            self.subgoal = candidate
        self.decision_goal_idx = self.subgoal
        metrics = evaluate_goal(self.motion, self.subgoal, self.evaluation_config)
        goal_s = self.motion.node_s[self.subgoal]
        radius = self.evaluation_config.neighborhood_radius
        forward = float(self.motion.probability @ self.motion.interval_weights(goal_s + radius, np.inf))
        backward = float(self.motion.probability @ self.motion.interval_weights(-np.inf, goal_s - self.recovery_distance))
        self.probabilities = dict(passed=metrics['local_passage_score'],
            near=metrics['position_score'], local=metrics['neighborhood_mass'],
            arrival=metrics['arrival_score'], forward=forward, backward=backward)
        is_final = self.subgoal == self.final_node
        if is_final:
            # Final arrival is a route-position area in every odometry mode;
            # the assumed heading is diagnostic only.
            near = self.motion.interval_weights(goal_s - self.final_tolerance, goal_s + self.final_tolerance)
            self.decision_score = float(self.motion.probability @ near)
        elif self.mode == 'arrival':
            self.decision_score = node_position_score(
                self.motion, self.subgoal, self.evaluation_config.position_tolerance)
            self.probabilities['arrival'] = self.decision_score
        else:
            self.decision_score = metrics['local_passage_score']
        if initial and not is_final:
            self.status = 'INITIAL'
            return self.subgoal, False
        if self.mode == 'nearest' and not is_final:
            self.status = 'NEAREST'
            return self.subgoal, False
        turn_ok = is_final or not self.turn_enabled or self._turn_satisfied()
        if metrics['neighborhood_mass'] < self.min_local_mass:
            self.count = 0
            self.status = 'REMOTE_BELIEF'
        elif self.decision_score < self.threshold:
            self.count = 0
            self.status = 'WAITING'
        elif not turn_ok:
            self.count = 0
            self.status = 'WAITING_TURN'
        else:
            self.count += 1
            self.status = 'CONFIRMING'
            # The posterior already integrates image/odom history. A short
            # target-node arrival window needs one fresh image; recovery and
            # final stopping retain their configured consecutive confirmation.
            required = 1 if self.mode == 'arrival' and not is_final else self.confirmations
            if self.count >= required:
                self.reset_confirmation()
                if is_final:
                    self.reached = True
                    self.status = 'GOAL_REACHED'
                else:
                    self._set_turn_anchor(self.subgoal)
                    self.subgoal += 1
                    self.status = 'ADVANCED_ARRIVAL' if self.mode == 'arrival' else 'ADVANCED_PASSAGE'
                return self.subgoal, self.reached
        if self.mode == 'nearest':
            return self.subgoal, False
        # A goal can be ahead by a normal image spacing, but should not remain
        # behind a passed neighborhood. All distances are physical route meters.
        candidate = min(closest_node + self.lookahead, self.final_node)
        direction = ('FORWARD' if turn_ok and candidate > self.subgoal and forward >= self.recovery_threshold
                     else 'BACKWARD' if candidate < self.subgoal and backward >= self.recovery_threshold
                     else None)
        position = self.motion.node_s[closest_node]
        coherent = float(self.motion.probability @ self.motion.interval_weights(position - radius, position + radius))
        if direction is None or coherent < self.recovery_threshold:
            self.recovery_count = 0
            self.recovery_candidate = None
            return self.subgoal, False
        key = (candidate, direction)
        if key != self.recovery_candidate:
            self.recovery_candidate = key
            self.recovery_count = 0
        self.recovery_count += 1
        self.status = 'RECOVERY_PENDING_' + direction
        if self.recovery_count >= self.confirmations:
            self.subgoal = candidate
            self.turn_anchor = None  # Recovery is not a confirmed arrival.
            self.reset_confirmation()
            self.status = 'RECOVERED_' + direction
        # Recovery is not an arrival observation and cannot terminate navigation.
        return self.subgoal, False
