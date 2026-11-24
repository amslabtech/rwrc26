"""Metric goal selection: arrival, passage, debounce and explicit recovery."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from placenav.topomap_odometry import OdomSample
from placenav.goal_evaluation import GoalEvaluationConfig
from placenav.place_recognition.odometry_motion import OdometryMotionModel
from placenav.subgoal_selector import ProgressSubgoalSelector


class SelectorTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.make_map([3.5*i for i in range(5)])
        self.selector = ProgressSubgoalSelector(self.motion, 4, mode='passage')

    def tearDown(self):
        self.temp.cleanup()

    def make_map(self, positions, yaws=None):
        names = [str(i)+'.jpg' for i in range(len(positions))]
        path = Path(self.temp.name)/'nodes.json'
        yaws = yaws or [0]*len(positions)
        path.write_text(json.dumps(dict(schema_version=1,
            units=dict(position='m', angle='rad'), child_frame_id='base_link',
            nodes=[dict(id=i, image=name, pose=dict(x=x, y=0, yaw=yaw))
                   for i, (name, x, yaw) in enumerate(zip(names, positions, yaws))])))
        self.motion = OdometryMotionModel(path, names, noise=0)
        self.motion.initialize(np.ones(len(names))/len(names))

    def mass(self, *positions):
        self.motion.probability.fill(0)
        for position, weight in positions:
            i = np.argmin(np.abs(self.motion.grid_s-position))
            self.motion.probability[i] += weight

    def test_turn_arrival_uses_signed_delta_from_confirmed_node(self):
        for direction in (1, -1):
            with self.subTest(direction=direction):
                angles = [direction * a for a in [0, 20, 40, 60, 60, 60]]
                self.make_map([0, 3, 6, 9, 12, 15], [math.radians(a) for a in angles])
                sel = ProgressSubgoalSelector(self.motion, 5, mode='arrival_turn')
                def step(stamp, node, yaw):
                    self.mass((3 * node, 1))
                    self.motion.last_odometry = OdomSample(
                        stamp, (0, 0, math.radians(yaw * direction)), 'odom', 'base_link')
                    return sel.update(node, stamp)
                step(1, 0, 175)
                self.assertEqual(step(2, 1, 179), (2, False))
                self.assertEqual(step(3, 2, 174), (3, False))  # Preview next turn image.
                self.assertEqual(step(4, 3, 174), (3, False))
                self.assertEqual(sel.status, 'WAITING_TURN')
                step(5, 4, 174)
                self.assertEqual(step(6, 4, 174), (3, False))
                self.assertEqual(step(7, 3, -158), (4, False))  # +23° from entry.

    def test_early_rotation_is_remembered_only_within_the_same_bend(self):
        self.make_map([0, 1, 2, 3, 4, 5], list(map(math.radians, [0, 20, 40, 60, 40, 20])))
        sel = ProgressSubgoalSelector(self.motion, 5, mode='arrival_turn',
                                     turn_check_start_deg=20, turn_required_ratio=.5)
        def step(stamp, closest, yaw):
            self.mass((closest, 1))
            self.motion.last_odometry = OdomSample(stamp, (0, 0, math.radians(yaw)), 'odom', 'base_link')
            return sel.update(closest, stamp)
        step(1, 0, 0)
        step(2, 1, 0)
        step(3, 1, 27)  # Started turning before reaching node 2.
        step(4, 2, 20)
        self.assertEqual(step(5, 3, -2), (3, False))  # Undid the turn.
        self.assertEqual(step(6, 3, 8), (4, False))  # Within tolerance again.
        step(7, 4, 8)  # First turning node of the next (right-hand) bend.
        self.assertEqual(sel.turn_anchor[0], 4)
        self.assertEqual(sel.peak_signed_turn, 0.)

    def test_turn_forward_recovery_exposes_one_goal(self):
        sel = ProgressSubgoalSelector(self.motion, 4, mode='arrival_turn')
        self.mass((0, 1))
        sel.update(0, 1)
        self.mass((10.5, 1))
        sel.update(3, 2)
        self.assertEqual(sel.update(3, 3), (2, False))
        self.assertEqual(sel.status, 'RECOVERED_FORWARD')

    def test_step_turn_arrival_uses_signed_delta_from_confirmed_node(self):
        for direction in (1, -1):
            with self.subTest(direction=direction):
                angles = [0, 0, 20 * direction, 20 * direction, 20 * direction]
                self.make_map([0, 3, 6, 9, 12], [math.radians(a) for a in angles])
                sel = ProgressSubgoalSelector(self.motion, 4, mode='arrival_turn_step')
                def step(stamp, node, yaw):
                    self.mass((3 * node, 1))
                    self.motion.last_odometry = OdomSample(
                        stamp, (0, 0, math.radians(yaw * direction)), 'odom', 'base_link')
                    return sel.update(node, stamp)
                step(1, 0, 175)
                self.assertEqual(step(2, 1, 179), (2, False))
                self.assertEqual(step(3, 2, 174), (2, False))
                self.assertEqual(sel.status, 'WAITING_TURN')
                step(4, 3, 174)
                self.assertEqual(step(5, 3, 174), (2, False))  # No forward rescue.
                self.assertEqual(step(6, 2, -169), (3, False))  # +12° across wrap.

    def test_step_turn_forward_recovery_uses_full_candidate(self):
        sel = ProgressSubgoalSelector(self.motion, 4, mode='arrival_turn_step')
        self.mass((0, 1))
        sel.update(0, 1)
        self.mass((10.5, 1))
        sel.update(3, 2)
        self.assertEqual(sel.update(3, 3), (4, False))
        self.assertEqual(sel.status, 'RECOVERED_FORWARD')

    def test_initial_goal_immediate_even_with_ambiguous_global_belief(self):
        self.mass((0, .5), (14, .5))
        for mode in ('nearest', 'passage', 'arrival'):
            selector = ProgressSubgoalSelector(self.motion, 4, mode=mode)
            self.assertEqual(selector.update(0, 1), (1, False))

    def test_midpoint_and_closest_changes_do_not_advance_retained_goals(self):
        for mode in ('passage', 'arrival'):
            selector = ProgressSubgoalSelector(self.motion, 4, mode=mode)
            self.mass((0, 1));selector.update(0, 1)
            self.mass((1.8, 1))
            for stamp, closest in enumerate([1, 0, 1, 1], 2):
                self.assertEqual(selector.update(closest, stamp), (1, False))

    def test_passage_requires_two_fresh_images_and_resets_after_failure(self):
        self.mass((0, 1));self.selector.update(0, 1)
        self.mass((3.6, 1))
        self.assertEqual(self.selector.update(1, 2), (1, False))
        self.assertEqual(self.selector.update(1, 2), (1, False))
        self.mass((3.4, 1));self.selector.update(1, 3)
        self.mass((3.6, 1));self.selector.update(1, 4)
        self.assertEqual(self.selector.update(1, 5), (2, False))
        self.assertEqual(self.selector.status, 'ADVANCED_PASSAGE')
        self.assertEqual(self.selector.decision_goal_idx, 1)
        self.assertEqual(self.selector.decision_score, 1)

    def test_small_backward_id_jitter_does_not_change_held_goal(self):
        self.mass((3.5, 1));self.selector.update(1, 1)  # Goal 2, 7 meters.
        self.mass((3.1, 1))
        for stamp in range(2, 7):
            self.assertEqual(self.selector.update(0, stamp), (2, False))

    def test_persistent_large_backward_error_recovers(self):
        for mode in ('passage', 'arrival'):
            selector = ProgressSubgoalSelector(self.motion, 4, mode=mode)
            self.mass((7, 1));selector.update(2, 1)
            self.mass((0, 1))
            self.assertEqual(selector.update(0, 2), (3, False))
            self.assertEqual(selector.status, 'RECOVERY_PENDING_BACKWARD')
            self.assertEqual(selector.update(0, 2), (3, False))
            self.assertEqual(selector.update(0, 3), (1, False))
            self.assertEqual(selector.status, 'RECOVERED_BACKWARD')

    def test_multimodal_far_tail_cannot_be_normal_passage_or_recovery(self):
        self.mass((0, 1));self.selector.update(0, 1)
        for distribution in [((0, .3), (3.4, .3), (7, .4)),
                             ((3.6, .55), (14, .45)),
                             ((0, .4), (3.6, .2), (7, .2), (10.5, .2))]:
            self.mass(*distribution)
            for stamp in range(self.selector.last_stamp+1, self.selector.last_stamp+5):
                self.assertEqual(self.selector.update(2, stamp), (1, False))
                self.assertEqual(self.selector.status, 'REMOTE_BELIEF')

    def test_large_forward_jump_is_recovery_not_arrival(self):
        self.mass((0, 1));self.selector.update(0, 1)
        self.mass((14, 1))
        self.assertEqual(self.selector.update(4, 2), (1, False))
        self.assertEqual(self.selector.update(4, 3), (4, False))
        self.assertEqual(self.selector.status, 'RECOVERED_FORWARD')
        self.assertEqual(self.selector.count, 0)
        self.assertEqual(self.selector.update(4, 3), (4, False))
        self.assertEqual(self.selector.update(4, 4), (4, False))
        self.assertEqual(self.selector.update(4, 5), (4, True))

    def test_recovery_requires_same_candidate_and_resets_on_disagreement(self):
        self.mass((0, 1));self.selector.update(0, 1)
        self.mass((7, 1));self.selector.update(2, 2)
        self.mass((10.5, 1));self.selector.update(3, 3)
        self.mass((7, 1));self.selector.update(2, 4)
        self.assertEqual(self.selector.subgoal, 1)
        self.assertEqual(self.selector.update(2, 5), (3, False))

    def test_recovery_requires_mass_near_candidate_not_only_a_high_argmax(self):
        self.mass((0, 1));self.selector.update(0, 1)
        self.mass((7, .5), (14, .5))
        for stamp in range(2, 7):
            self.assertEqual(self.selector.update(2, stamp), (1, False))
            self.assertEqual(self.selector.recovery_count, 0)

    def test_final_goal_can_recover_back_before_arrival(self):
        self.mass((10.5, 1));self.selector.update(3, 1)
        self.mass((3.5, 1));self.selector.update(1, 2)
        self.assertEqual(self.selector.update(1, 3), (2, False))
        self.assertEqual(self.selector.status, 'RECOVERED_BACKWARD')

    def test_recovery_never_exceeds_configured_final_goal(self):
        selector = ProgressSubgoalSelector(self.motion, 2, mode='passage')
        self.mass((0, 1));selector.update(0, 1)
        self.mass((14, 1))
        self.assertEqual(selector.update(4, 2), (1, False))
        for stamp in range(3, 7):
            self.assertEqual(selector.update(4, stamp), (2, False))

    def test_confirmation_reset_and_clock_rewind_keep_goal(self):
        self.mass((0, 1));self.selector.update(0, 10)
        self.mass((3.6, 1));self.selector.update(1, 11)
        self.selector.reset_confirmation()
        self.assertEqual(self.selector.update(1, 12), (1, False))
        self.assertEqual(self.selector.update(1, 5), (1, False))
        self.assertEqual(self.selector.update(1, 6), (2, False))

    def test_nearest_keeps_original_image_selection(self):
        selector = ProgressSubgoalSelector(self.motion, 4)
        self.mass((0, 1))
        for stamp, closest in enumerate([0, 1, 0, 2], 1):
            self.assertEqual(selector.update(closest, stamp), (closest+1, False))

    def test_all_modes_final_position_confirmation_and_terminal_latch(self):
        for mode in ('nearest', 'passage', 'arrival'):
            selector = ProgressSubgoalSelector(self.motion, 4, mode=mode)
            self.motion.heading_offset.fill(math.pi)
            self.mass((12.3, 1))
            for stamp in range(1, 4):
                self.assertEqual(selector.update(4, stamp), (4, False))
            self.mass((13.8, 1))
            self.assertEqual(selector.update(4, 4), (4, False))
            self.assertEqual(selector.update(4, 5), (4, True))
            self.mass((0, 1))
            self.assertEqual(selector.update(0, 6), (4, True))

    def test_final_needs_global_local_mass_not_conditioned_mass(self):
        self.mass((14, .6), (0, .4))
        for stamp in range(1, 5):
            self.assertEqual(self.selector.update(4, stamp), (4, False))
            self.assertEqual(self.selector.status, 'REMOTE_BELIEF')

    def test_intermediate_final_goal_does_not_accept_farther_positions(self):
        for mode in ('nearest', 'passage', 'arrival'):
            selector = ProgressSubgoalSelector(self.motion, 2, mode=mode)
            self.mass((9, 1))
            for stamp in range(1, 4):
                self.assertEqual(selector.update(2, stamp), (2, False))
            self.mass((6.8, 1));selector.update(2, 4)
            self.assertEqual(selector.update(2, 5), (2, True))

    def test_arrival_uses_one_fresh_posterior_without_a_heading_gate(self):
        selector = ProgressSubgoalSelector(self.motion, 4, mode='arrival')
        self.mass((0, 1));selector.update(0, 1)
        self.mass((3.2, 1));self.motion.heading_offset.fill(math.pi/2)
        self.assertEqual(selector.update(1, 2), (2, False))
        self.assertEqual(selector.status, 'ADVANCED_ARRIVAL')
        self.assertEqual(selector.decision_goal_idx, 1)
        self.assertEqual(selector.decision_score, 1)
        self.assertEqual(selector.count, 0)
        self.mass((6.8, 1))
        self.assertEqual(selector.update(2, 2), (2, False))
        self.assertEqual(selector.update(2, 3), (3, False))

    def test_final_arrival_ignores_heading_but_keeps_area_confirmation_and_latch(self):
        selector = ProgressSubgoalSelector(self.motion, 4, mode='arrival')
        self.mass((13.8, 1));self.motion.heading_offset.fill(math.pi/2)
        self.assertEqual(selector.update(4, 1), (4, False))
        self.assertEqual(selector.update(4, 1), (4, False))
        self.mass((12.3, 1))
        self.assertEqual(selector.update(4, 2), (4, False))
        self.mass((13.8, 1))
        self.assertEqual(selector.update(4, 3), (4, False))
        self.motion.heading_offset.fill(-math.pi)
        self.assertEqual(selector.update(4, 4), (4, True))
        self.assertEqual(selector.update(4, 4), (4, True))
        self.mass((0, 1))
        self.assertEqual(selector.update(0, 5), (4, True))

    def test_arrival_requires_global_neighborhood_mass(self):
        selector = ProgressSubgoalSelector(self.motion, 4, mode='arrival')
        self.mass((0, 1));selector.update(0, 1)
        self.mass((3.5, .6), (14, .4))
        for stamp in range(2, 5):
            self.assertEqual(selector.update(1, stamp), (1, False))
            self.assertEqual(selector.status, 'REMOTE_BELIEF')

    def test_arrival_missed_during_avoidance_can_recover_forward(self):
        selector = ProgressSubgoalSelector(self.motion, 4, mode='arrival')
        self.mass((0, 1));selector.update(0, 1)
        self.mass((7, 1));self.motion.heading_offset.fill(math.pi/2)
        selector.update(2, 2)
        self.assertEqual(selector.update(2, 3), (3, False))
        self.assertEqual(selector.status, 'RECOVERED_FORWARD')

    def test_dense_turn_does_not_advance_on_nearby_other_node_support(self):
        self.make_map([0, .2, .4, 3.9, 7.4], [0, math.pi/2, math.pi, math.pi, math.pi])
        selector = ProgressSubgoalSelector(self.motion, 4, mode='arrival')
        self.mass((0, 1));selector.update(0, 1)
        selector.update(0, 2);selector.update(0, 3)
        self.assertEqual(selector.subgoal, 1)  # Position overlaps; target owner does not.
        self.mass((.2, 1))
        self.motion.heading_offset[:] = -self.motion.yaw
        selector.update(1, 4)
        self.assertEqual(selector.subgoal, 2)
        selector.update(1, 4);selector.update(1, 5)
        self.assertEqual(selector.subgoal, 2)
        self.mass((.4, 1))
        self.assertEqual(selector.update(2, 6), (3, False))

    def test_selection_and_evaluation_do_not_change_localization(self):
        self.mass((3.6, .2), (14, .8))
        original = [x.copy() for x in (self.motion.probability, self.motion.grid_s,
                                      self.motion.heading_offset, self.motion.owner)]
        for mode in ('passage', 'arrival'):
            selector = ProgressSubgoalSelector(self.motion, 4, mode=mode)
            selector.update(0, 1);selector.update(1, 2)
        for before, after in zip(original, (self.motion.probability, self.motion.grid_s,
                                           self.motion.heading_offset, self.motion.owner)):
            np.testing.assert_array_equal(before, after)

    def test_invalid_configuration_is_rejected(self):
        for options in [dict(mode='typo'), dict(mode='passage', lookahead=2),
                        dict(mode='arrival', lookahead=0), dict(threshold=0),
                        dict(threshold=float('nan')), dict(min_local_mass=1.1),
                        dict(confirmations=0), dict(final_tolerance=-1),
                        dict(recovery_distance=1), dict(recovery_threshold=0)]:
            with self.subTest(options=options), self.assertRaises(ValueError):
                ProgressSubgoalSelector(self.motion, 4, **options)


if __name__ == '__main__':
    unittest.main()
