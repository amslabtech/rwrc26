"""Metric goal decisions from the posterior, independent of ROS and models."""
import math

import numpy as np

from placenav.goal_evaluation import GoalEvaluationConfig, evaluate_goal, node_position_score


# A 15-degree teaching interval is also the minimum observed turn and the
# allowed angular shortfall after qualifying earlier in this bend.
TURN_STEP_RAD = math.radians(15)


class ProgressSubgoalSelector:
    def __init__(self, motion, final_node, mode='nearest', lookahead=1,
                 threshold=0.5, confirmations=2, min_local_mass=0.8,
                 final_tolerance=0.5, evaluation_config=None,
                 recovery_distance=4.0, recovery_threshold=0.8,
                 turn_check_start_deg=30., turn_required_ratio=0.25):
        # arrival_turn shares all arrival behavior except the relative-yaw gate.
        self.turn_enabled = mode == 'arrival_turn'
        self.turn_anchor = None
        if self.turn_enabled:
            if not math.isfinite(turn_check_start_deg) or turn_check_start_deg <= 0:
                raise ValueError('turn_check_start_deg must be positive')
            if not 0 < turn_required_ratio <= 1:
                raise ValueError('turn_required_ratio must be in (0, 1]')
            self.turn_check_start = math.radians(turn_check_start_deg)
            self.turn_required_ratio = turn_required_ratio
            self.map_yaw = np.unwrap(motion.node_yaw)
            changes = np.diff(self.map_yaw, prepend=self.map_yaw[0])
            direction = np.where(np.abs(changes) >= TURN_STEP_RAD, np.sign(changes), 0)
            self.turn_entry = []
            for node, sign in enumerate(direction):
                entry = self.turn_entry[-1] if node and sign and sign == direction[node-1] else node
                self.turn_entry.append(entry)
            mode = 'arrival'
        self.previous_yaw = None
        self.turn_yaw = 0.
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
        self.previous_yaw = None
        self.turn_yaw = 0.
        self.reset_confirmation()

    def _observe_yaw(self):
        sample = self.motion.last_odometry
        if sample is None:
            return
        yaw = sample.pose[2]
        if self.previous_yaw is not None:
            delta = yaw - self.previous_yaw
            self.turn_yaw += math.atan2(math.sin(delta), math.cos(delta))
        self.previous_yaw = yaw

    def _set_turn_anchor(self, node):
        if not self.turn_enabled or self.previous_yaw is None:
            return
        if self.turn_anchor is None or self.turn_entry[node] != self.turn_entry[self.turn_anchor[0]]:
            self.turn_anchor = (node, self.turn_yaw)
            self.peak_signed_turn = 0.

    def _turn_satisfied(self):
        if self.turn_anchor is None:
            return True
        node, start_yaw = self.turn_anchor
        if self.turn_entry[node] != self.turn_entry[self.subgoal]:
            return True
        taught = self.map_yaw[self.subgoal] - self.map_yaw[node]
        measured = math.copysign(1., taught) * (self.turn_yaw - start_yaw)
        self.peak_signed_turn = max(self.peak_signed_turn, measured)
        if abs(taught) < self.turn_check_start:
            return True
        required = max(TURN_STEP_RAD, self.turn_required_ratio * abs(taught))
        return self.peak_signed_turn >= required and measured >= required - TURN_STEP_RAD

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
        if self.turn_enabled:
            self._observe_yaw()
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
            if self.turn_enabled and direction == 'FORWARD':
                # Keep the turn reference and expose one new goal per recovery.
                self._set_turn_anchor(self.subgoal)
                self.subgoal += 1
            else:
                self.subgoal = candidate
                self.turn_anchor = None  # Backward recovery starts a new approach.
            self.reset_confirmation()
            self.status = 'RECOVERED_' + direction
        # Recovery is not an arrival observation and cannot terminate navigation.
        return self.subgoal, False
