"""Planar propagation and position-only arrival/recovery anchors."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from placenav.local_pose import LocalPoseConfig, LocalPoseTracker
from placenav.place_recognition.odometry_motion import OdometryHistory, OdometryMotionModel
from placenav.topomap_odometry import OdomSample, relative_pose
from placenav.subgoal_selector import ProgressSubgoalSelector


def sample(t, x=0, y=0, yaw=0, frame='odom'):
    return OdomSample(round(t*1e9), (x, y, yaw), frame, 'base_link')


class LocalPoseTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name)/'nodes.json'
        names = [str(i)+'.jpg' for i in range(5)]
        self.path.write_text(json.dumps(dict(schema_version=1,
            units=dict(position='m', angle='rad'), child_frame_id='base_link',
            nodes=[dict(id=i, image=name, pose=dict(x=2*i, y=0, yaw=0)) for i, name in enumerate(names)])))
        self.motion = OdometryMotionModel(self.path, names, noise=0)
        self.motion.initialize(np.ones(5)/5)
        self.point(2)

    def tearDown(self):
        self.temp.cleanup()

    def point(self, position):
        self.motion.probability.fill(0)
        i = int(np.argmin(abs(self.motion.grid_s-position)))
        self.motion.grid_s[i] = position
        self.motion._update_geometry()
        self.motion.probability[i] = 1

    def tracker(self, **kwargs):
        return LocalPoseTracker(self.motion, LocalPoseConfig(**kwargs))

    def test_initial_pose_is_conditional_not_average_of_remote_hypotheses(self):
        self.point(2)
        self.motion.probability *= .6
        self.motion.probability[np.argmin(abs(self.motion.grid_s-8))] = .4
        tracker = self.tracker();tracker.update(1, sample(10), 0)
        np.testing.assert_allclose(tracker.mean, [2, 0, 0])
        self.assertAlmostEqual(tracker.anchor_probability, .6)
        self.assertGreater(tracker.covariance[1, 1], 0)
        self.assertGreater(tracker.covariance[2, 2], 0)

    def test_goal_pose_is_forward_left_and_relative_heading(self):
        tracker = self.tracker();tracker.update(1, sample(10), 0)
        tracker.update(1, sample(11, yaw=math.pi/2), 0)
        goal, _ = tracker.goal(2)
        np.testing.assert_allclose(goal, [0, -2, -math.pi/2], atol=1e-12)

    def test_avoidance_tracks_lateral_offset_and_returns_without_reset(self):
        tracker = self.tracker();tracker.update(1, sample(10), 0)
        tracker.update(1, sample(11, y=1), 0)
        np.testing.assert_allclose(tracker.goal(2)[0], [2, -1, 0])
        tracker.update(1, sample(12, x=1, y=1), 0)
        tracker.update(1, sample(13, x=1, y=0), 0)
        np.testing.assert_allclose(tracker.mean, [3, 0, 0])
        np.testing.assert_allclose(tracker.goal(2)[0], [1, 0, 0])

    def test_long_publication_gap_keeps_translation_heading_and_local_track(self):
        tracker = self.tracker()
        history = OdometryHistory(.2, max_speed=3)
        epoch = None
        for current in [sample(10), sample(10.1, yaw=math.pi/2),
                        sample(15, y=1, yaw=math.pi/2), sample(15.1, y=1.2, yaw=math.pi/2)]:
            history.add(current)
            if epoch != history.epoch:
                self.motion.reset_odometry();epoch = history.epoch
            image_odom = history.at(current.stamp_ns)
            self.motion.predict(image_odom)
            tracker.update(1, image_odom, epoch)
        self.assertEqual(history.epoch, 0)
        self.assertIsNone(history.at(12_000_000_000))
        self.assertTrue(tracker.continuous)
        np.testing.assert_allclose(tracker.mean, [2, 1.2, math.pi/2], atol=1e-12)
        # The robot's heading remains 90 degrees despite the long input gap.
        occupied = self.motion.probability > 0
        heading = self.motion.yaw[occupied]+self.motion.heading_offset[occupied]
        np.testing.assert_allclose(heading, math.pi/2, atol=1e-12)

    def test_live_origin_and_heading_can_differ_from_map(self):
        a, b = self.tracker(), self.tracker()
        a.update(1, sample(10), 0)
        angle = 1.2
        b.update(1, sample(10, 100, -20, angle, 'vo'), 0)
        a.update(1, sample(11, .3, .4, .2), 0)
        b.update(1, sample(11, 100+.3*math.cos(angle)-.4*math.sin(angle),
                          -20+.3*math.sin(angle)+.4*math.cos(angle), angle+.2, 'vo'), 0)
        np.testing.assert_allclose(a.mean, b.mean, atol=1e-12)
        np.testing.assert_allclose(a.covariance, b.covariance, atol=1e-12)

    def test_goal_switch_or_vpr_jump_does_not_change_pose(self):
        tracker = self.tracker();tracker.update(1, sample(10), 0)
        mean, covariance = tracker.mean.copy(), tracker.covariance.copy()
        for goal in [2, 4, 1, 3]:
            tracker.goal(goal)
        self.point(8)
        tracker.update(4, sample(11), 0)
        np.testing.assert_array_equal(mean, tracker.mean)
        np.testing.assert_array_equal(covariance, tracker.covariance)
        self.assertAlmostEqual(tracker.belief_disagreement()[0], 6)
        self.assertEqual(tracker.anchor_node, 1)

    def arrival(self, tracker, yaw=.7):
        selector = ProgressSubgoalSelector(self.motion, 4, mode='arrival')
        selector.update(0, 10_000_000_000)  # Initial goal is node 1.
        tracker.update(1, sample(10), 0)
        tracker.update(1, sample(11, .4, .2, yaw), 0)
        selector.update(1, 11_000_000_000)
        self.assertEqual(selector.status, 'ADVANCED_ARRIVAL')
        return selector

    def test_arrival_resets_reached_xy_and_preserves_heading_and_variance(self):
        tracker = self.tracker()
        selector = self.arrival(tracker)
        heading_variance = tracker.covariance[2, 2]
        self.assertTrue(tracker.correct_arrival(selector))
        np.testing.assert_allclose(tracker.mean, [2, 0, .7])
        np.testing.assert_allclose(tracker.goal(2)[0],
                                   [2*math.cos(.7), -2*math.sin(.7), -.7])
        self.assertEqual(tracker.anchor_node, 1)  # Not the new goal, node 2.
        self.assertEqual(tracker.anchor_stamp_ns, 11_000_000_000)
        self.assertEqual(tracker.arrival_count, 1)
        self.assertAlmostEqual(tracker.covariance[2, 2], heading_variance)
        np.testing.assert_allclose(tracker.covariance[:2, :2], np.eye(2)*.25)
        np.testing.assert_allclose(tracker.covariance[:2, 2], 0)
        self.assertGreaterEqual(np.linalg.eigvalsh(tracker.covariance).min(), 0)
        self.assertEqual(tracker.status, 'ARRIVAL_POSITION_RESET')

    def test_three_modes_with_reset_on_or_off_preserve_pose_output_and_filter(self):
        for mode in ['nearest', 'passage', 'arrival']:
            for enabled in [False, True]:
                with self.subTest(mode=mode, enabled=enabled):
                    self.point(0)
                    selector = ProgressSubgoalSelector(self.motion, 4, mode=mode)
                    selector.update(0, 10_000_000_000)
                    tracker = self.tracker(); tracker.update(0, sample(10), 0)
                    self.assertFalse(tracker.correct_subgoal_switch(
                        selector, None, 0, enabled=enabled))
                    for t in ([11, 12] if mode == 'passage' else [11]):
                        self.point(2.1)
                        tracker.update(1, sample(t, 2.4, .2, .7), 0)
                        previous_goal = selector.subgoal
                        selector.update(1, t*1_000_000_000)
                        before, covariance = tracker.mean.copy(), tracker.covariance.copy()
                        arrays = [x.copy() for x in (self.motion.probability, self.motion.grid_s,
                            self.motion.heading_offset, self.motion.position_variance)]
                        changed = selector.subgoal != previous_goal
                        self.assertEqual(tracker.correct_subgoal_switch(
                            selector, previous_goal, 1, enabled=enabled), enabled and changed)
                        if not enabled or not changed:
                            np.testing.assert_array_equal(tracker.mean, before)
                            np.testing.assert_array_equal(tracker.covariance, covariance)
                        self.assertEqual(tracker.covariance[2, 2], covariance[2, 2])
                        self.assertEqual(tracker.mean[2], before[2])
                        for a, b in zip(arrays, (self.motion.probability, self.motion.grid_s,
                                self.motion.heading_offset, self.motion.position_variance)):
                            np.testing.assert_array_equal(a, b)
                    self.assertEqual(selector.subgoal, 2)
                    self.assertEqual(tracker.anchor_node, 1 if enabled else 0)
                    self.assertEqual(tracker.switch_count, int(enabled))
                    expected = np.array([2., 0.]) if enabled else np.array([2.4, .2])
                    np.testing.assert_allclose(tracker.mean[:2], expected)
                    goal, covariance = tracker.goal(2)
                    self.assertTrue(np.isfinite(goal).all())
                    self.assertGreaterEqual(np.linalg.eigvalsh(covariance).min(), -1e-10)
                    if enabled:
                        self.assertEqual(tracker.status, mode.upper()+'_POSITION_RESET')
                        self.assertFalse(tracker.correct_subgoal_switch(
                            selector, previous_goal, 1))  # The same event cannot reset twice.
                    tracker.update(1, sample(13, 2.4+math.cos(.7), .2+math.sin(.7), .7), 0)
                    np.testing.assert_allclose(tracker.mean[:2], expected+[math.cos(.7), math.sin(.7)])

    def test_nearest_reset_uses_current_node_with_lookahead_backward_and_large_changes(self):
        for before, after, lookahead in [(0, 1, 2), (3, 1, 1), (0, 3, 1)]:
            with self.subTest(before=before, after=after, lookahead=lookahead):
                self.point(2*before)
                selector = ProgressSubgoalSelector(self.motion, 4, mode='nearest', lookahead=lookahead)
                selector.update(before, 10_000_000_000)
                tracker = self.tracker(); tracker.update(before, sample(10), 0)
                previous_goal = selector.subgoal
                self.point(2*after)
                selector.update(after, 11_000_000_000)
                tracker.update(after, sample(11, .4, .2, .3), 0)
                self.assertTrue(tracker.correct_subgoal_switch(selector, previous_goal, after))
                np.testing.assert_allclose(tracker.mean, [2*after, 0, .3])
                self.assertEqual(tracker.anchor_node, after)

    def test_nearest_reset_rejects_stale_discontinuous_and_terminal_events(self):
        self.point(0)
        selector = ProgressSubgoalSelector(self.motion, 4, mode='nearest')
        selector.update(0, 10_000_000_000)
        tracker = self.tracker(); tracker.update(0, sample(10), 0)
        self.point(2)
        selector.update(1, 11_000_000_000)
        self.assertFalse(tracker.correct_subgoal_switch(selector, 1, 1))  # Stale local pose.
        tracker.update(1, sample(11, .4), 0)
        before = tracker.mean.copy()
        selector.reached = True
        self.assertFalse(tracker.correct_subgoal_switch(selector, 1, 1))
        selector.reached = False
        tracker.invalidate('ODOMETRY_DISCONTINUITY')
        self.assertFalse(tracker.correct_subgoal_switch(selector, 1, 1))
        np.testing.assert_array_equal(tracker.mean, before)

    def test_arrival_continues_from_current_live_odom_without_reusing_movement(self):
        tracker = self.tracker()
        selector = self.arrival(tracker)
        self.assertTrue(tracker.correct_arrival(selector))
        tracker.update(2, sample(12, .4+math.cos(.7), .2+math.sin(.7), .7), 0)
        np.testing.assert_allclose(tracker.mean, [2+math.cos(.7), math.sin(.7), .7])
        self.assertEqual(tracker.status, 'TRACKING_POSITION_ANCHOR')

    def test_arrival_at_same_timestamp_cannot_reanchor_twice(self):
        tracker = self.tracker()
        selector = self.arrival(tracker)
        self.assertTrue(tracker.correct_arrival(selector))
        covariance = tracker.covariance.copy()
        self.assertFalse(tracker.correct_arrival(selector))
        self.assertEqual(tracker.arrival_count, 1)
        np.testing.assert_array_equal(tracker.covariance, covariance)

    def test_arrival_correction_only_accepts_ordinary_arrival(self):
        tracker = self.tracker()
        selector = self.arrival(tracker)
        before = tracker.mean.copy()
        for status in ['INITIAL', 'NEAREST', 'WAITING', 'ADVANCED_PASSAGE',
                       'RECOVERED_FORWARD', 'RECOVERED_BACKWARD', 'GOAL_REACHED']:
            selector.status = status
            self.assertFalse(tracker.correct_arrival(selector))
        selector.status = 'ADVANCED_ARRIVAL'
        for mode in ['nearest', 'passage']:
            selector.mode = mode
            self.assertFalse(tracker.correct_arrival(selector))
        np.testing.assert_array_equal(tracker.mean, before)

    def test_arrival_rejects_stale_or_mismatched_decision(self):
        tracker = self.tracker()
        selector = self.arrival(tracker)
        before = tracker.mean.copy()
        selector.last_stamp -= 1
        self.assertFalse(tracker.correct_arrival(selector))
        selector.last_stamp += 1
        selector.subgoal = 4
        self.assertFalse(tracker.correct_arrival(selector))
        selector.subgoal = 2
        selector.decision_goal_idx = -1
        self.assertFalse(tracker.correct_arrival(selector))
        np.testing.assert_array_equal(tracker.mean, before)

    def test_arrival_never_revives_discontinuous_track(self):
        tracker = self.tracker()
        selector = self.arrival(tracker)
        tracker.update(1, sample(12), 1)
        before = tracker.mean.copy()
        selector.last_stamp = 12_000_000_000
        self.assertFalse(tracker.correct_arrival(selector))
        np.testing.assert_array_equal(tracker.mean, before)
        self.assertFalse(tracker.continuous)
        self.assertEqual(tracker.status, 'ODOMETRY_DISCONTINUITY')

    def test_arrival_correction_does_not_modify_filter(self):
        tracker = self.tracker()
        selector = self.arrival(tracker, yaw=math.pi-.01)
        arrays = [x.copy() for x in (self.motion.probability, self.motion.grid_s,
                                     self.motion.heading_offset, self.motion.position_variance)]
        self.assertTrue(tracker.correct_arrival(selector))
        self.assertAlmostEqual(tracker.mean[2], math.pi-.01)
        for before, after in zip(arrays, (self.motion.probability, self.motion.grid_s,
                                         self.motion.heading_offset, self.motion.position_variance)):
            np.testing.assert_array_equal(before, after)

    def recovery(self, tracker, forward=False, mode='arrival'):
        before, after = (0, 3) if forward else (2, 0)
        self.point(2*before)
        selector = ProgressSubgoalSelector(self.motion, 4, mode=mode)
        selector.update(before, 10_000_000_000)
        tracker.update(before, sample(10), 0)
        self.point(2*after+.4)  # Between recorded positions, not on the node.
        for t in [11, 12]:
            tracker.update(after, sample(t, .4, .2, .7), 0)
            selector.update(after, t*1_000_000_000)
        self.assertEqual(selector.status, 'RECOVERED_FORWARD' if forward else 'RECOVERED_BACKWARD')
        return selector, after

    def test_recovery_repairs_wrong_reference_using_local_posterior_not_next_goal(self):
        tracker = self.tracker()
        selector, node = self.recovery(tracker)
        # An unrelated remote mode must not pull the selected position away.
        self.motion.probability *= .85
        remote = int(np.argmin(abs(self.motion.grid_s-8)))
        self.motion.probability[remote] = .15
        self.motion.position_variance[remote] = 100.  # Even its broad tail must not pull the mean.
        yaw_variance = tracker.covariance[2, 2]
        self.assertTrue(tracker.correct_recovery(selector, node))
        np.testing.assert_allclose(tracker.mean, [.4, 0, .7])
        np.testing.assert_allclose(tracker.goal(1)[0],
                                   [1.6*math.cos(.7), -1.6*math.sin(.7), -.7])
        self.assertEqual(tracker.anchor_node, 0)
        self.assertEqual(selector.subgoal, 1)
        self.assertEqual(tracker.anchor_stamp_ns, 12_000_000_000)
        self.assertAlmostEqual(tracker.anchor_probability, .85)
        self.assertAlmostEqual(tracker.covariance[2, 2], yaw_variance)
        np.testing.assert_allclose(tracker.covariance[:2, 2], 0)
        self.assertGreaterEqual(np.linalg.eigvalsh(tracker.covariance).min(), 0)
        self.assertEqual(tracker.recovery_count, 1)
        self.assertEqual(tracker.arrival_count, 0)
        self.assertEqual(tracker.status, 'RECOVERY_POSITION_RESET')

    def test_forward_and_passage_recovery_can_repair_position(self):
        for forward, mode in [(True, 'arrival'), (False, 'passage'), (True, 'passage')]:
            tracker = self.tracker()
            selector, node = self.recovery(tracker, forward=forward, mode=mode)
            self.assertTrue(tracker.correct_recovery(selector, node))
            np.testing.assert_allclose(tracker.mean, [2*node+.4, 0, .7])

    def test_reset_off_also_disables_recovery_without_changing_its_goal(self):
        for mode in ['arrival', 'passage']:
            with self.subTest(mode=mode):
                tracker = self.tracker()
                selector, node = self.recovery(tracker, mode=mode)
                before, covariance = tracker.mean.copy(), tracker.covariance.copy()
                goal = selector.subgoal
                self.assertFalse(tracker.correct_recovery(selector, node, enabled=False))
                np.testing.assert_array_equal(tracker.mean, before)
                np.testing.assert_array_equal(tracker.covariance, covariance)
                self.assertEqual(tracker.recovery_count, 0)
                self.assertEqual(selector.subgoal, goal)
                self.assertTrue(tracker.correct_recovery(selector, node, enabled=True))

    def test_recovery_keeps_consistent_pose_and_small_avoidance_offsets(self):
        for forward in [False, True]:
            tracker = self.tracker()
            selector, node = self.recovery(tracker, forward=forward)
            tracker.mean[:2] = [2*node+.4, 1.]  # A one-meter lateral detour.
            mean, covariance = tracker.mean.copy(), tracker.covariance.copy()
            anchor = tracker.anchor_node
            self.assertFalse(tracker.correct_recovery(selector, node))
            np.testing.assert_array_equal(tracker.mean, mean)
            np.testing.assert_array_equal(tracker.covariance, covariance)
            self.assertEqual(tracker.anchor_node, anchor)
            self.assertEqual(tracker.recovery_count, 0)

    def test_recovery_uses_belief_spread_and_subcell_variance(self):
        tracker = self.tracker()
        selector, node = self.recovery(tracker)
        self.motion.probability *= .5
        second = int(np.argmin(abs(self.motion.grid_s-.8)))
        self.motion.probability[second] = .5
        self.motion.position_variance[self.motion.probability > 0] = .01
        self.assertTrue(tracker.correct_recovery(selector, node))
        np.testing.assert_allclose(tracker.mean[:2], [.6, 0], atol=1e-10)
        self.assertAlmostEqual(tracker.covariance[0, 0], .25+.04+.01, places=9)
        self.assertAlmostEqual(tracker.covariance[1, 1], .25)

    def test_recovery_requires_coherent_posterior(self):
        tracker = self.tracker()
        selector, node = self.recovery(tracker)
        self.motion.probability *= .7
        self.motion.probability[np.argmin(abs(self.motion.grid_s-8))] = .3
        before = tracker.mean.copy()
        self.assertFalse(tracker.correct_recovery(selector, node))
        np.testing.assert_array_equal(tracker.mean, before)

    def test_recovery_does_not_repeat_or_reintegrate_old_odometry(self):
        tracker = self.tracker()
        selector, node = self.recovery(tracker)
        self.assertTrue(tracker.correct_recovery(selector, node))
        self.assertFalse(tracker.correct_recovery(selector, node))
        next_odom = sample(13, .4+math.cos(.7), .2+math.sin(.7), .7)
        tracker.update(node, next_odom, 0)
        np.testing.assert_allclose(tracker.mean, [.4+math.cos(.7), math.sin(.7), .7])
        self.assertEqual(tracker.status, 'TRACKING_POSITION_ANCHOR')
        self.assertEqual(tracker.recovery_count, 1)

    def test_recovery_rejects_pending_stale_or_mismatched_decision(self):
        tracker = self.tracker()
        selector, node = self.recovery(tracker)
        before = tracker.mean.copy()
        for status in ['RECOVERY_PENDING_BACKWARD', 'WAITING', 'INITIAL', 'ADVANCED_ARRIVAL']:
            selector.status = status
            self.assertFalse(tracker.correct_recovery(selector, node))
        selector.status = 'RECOVERED_BACKWARD'
        selector.last_stamp -= 1
        self.assertFalse(tracker.correct_recovery(selector, node))
        selector.last_stamp += 1
        self.assertFalse(tracker.correct_recovery(selector, 3))
        selector.mode = 'nearest'
        self.assertFalse(tracker.correct_recovery(selector, node))
        np.testing.assert_array_equal(tracker.mean, before)

    def test_recovery_never_revives_discontinuous_track(self):
        tracker = self.tracker()
        selector, node = self.recovery(tracker)
        tracker.invalidate('ODOMETRY_DISCONTINUITY')
        before = tracker.mean.copy()
        self.assertFalse(tracker.correct_recovery(selector, node))
        np.testing.assert_array_equal(tracker.mean, before)
        self.assertFalse(tracker.continuous)

    def test_recovery_correction_does_not_modify_filter_or_goal(self):
        tracker = self.tracker()
        selector, node = self.recovery(tracker)
        arrays = [x.copy() for x in (self.motion.probability, self.motion.grid_s,
                                     self.motion.heading_offset, self.motion.position_variance)]
        goal, status = selector.subgoal, selector.status
        self.assertTrue(tracker.correct_recovery(selector, node))
        for before, after in zip(arrays, (self.motion.probability, self.motion.grid_s,
                                         self.motion.heading_offset, self.motion.position_variance)):
            np.testing.assert_array_equal(before, after)
        self.assertEqual((selector.subgoal, selector.status), (goal, status))

    def test_duplicate_and_stationary_updates_do_not_shrink_covariance(self):
        tracker = self.tracker();tracker.update(1, sample(10), 0)
        mean, covariance = tracker.mean.copy(), tracker.covariance.copy()
        tracker.update(1, sample(10), 0)
        tracker.update(1, sample(11), 0)
        np.testing.assert_array_equal(mean, tracker.mean)
        np.testing.assert_array_equal(covariance, tracker.covariance)

    def test_pure_turn_increases_heading_uncertainty(self):
        tracker = self.tracker();tracker.update(1, sample(10), 0)
        before = tracker.covariance[2, 2]
        tracker.update(1, sample(11, yaw=1), 0)
        np.testing.assert_allclose(tracker.mean, [2, 0, 1])
        self.assertGreater(tracker.covariance[2, 2], before)

    def test_straight_motion_covariance_is_consistent_under_subdivision(self):
        results = []
        for count in (1, 10, 100):
            tracker = self.tracker();tracker.update(1, sample(10), 0)
            for i in range(1, count+1):
                tracker.update(1, sample(10+i/count, i/count), 0)
            results.append((tracker.mean, tracker.covariance))
        for mean, covariance in results[1:]:
            np.testing.assert_allclose(mean, results[0][0], atol=1e-12)
            np.testing.assert_allclose(covariance, results[0][1], atol=1e-12)

    def test_discontinuities_freeze_track_until_restart(self):
        for changed, epoch in [(sample(11, x=100), 1), (sample(11, frame='vo'), 0),
                               (sample(9), 0), (sample(10, x=.1), 0)]:
            tracker = self.tracker();tracker.update(1, sample(10), 0)
            mean = tracker.mean.copy()
            tracker.update(4, changed, epoch)
            self.assertFalse(tracker.continuous)
            tracker.update(4, sample(12, x=100), epoch)
            np.testing.assert_array_equal(mean, tracker.mean)
            self.assertEqual(tracker.last_odometry.stamp_ns, 10_000_000_000)

    def test_goal_covariance_matches_numerical_transform_jacobian(self):
        tracker = self.tracker();tracker.update(1, sample(10), 0)
        tracker.update(1, sample(11, .5, -.3, .7), 0)
        _, covariance = tracker.goal(3)
        target = [6, 0, 0]
        jacobian = np.empty((3, 3))
        for j in range(3):
            offset = np.zeros(3);offset[j] = 1e-6
            jacobian[:, j] = (np.array(relative_pose(tracker.mean+offset, target)) -
                              relative_pose(tracker.mean-offset, target)) / 2e-6
        np.testing.assert_allclose(covariance, jacobian@tracker.covariance@jacobian.T, atol=1e-9)

    def test_endpoint_pose_extrapolates_instead_of_clipping(self):
        self.point(9)
        tracker = self.tracker();tracker.update(4, sample(10), 0)
        np.testing.assert_allclose(tracker.mean, [9, 0, 0])
        np.testing.assert_allclose(tracker.goal(4)[0], [-1, 0, 0])

    def test_covariance_stays_finite_symmetric_and_positive(self):
        tracker = self.tracker();tracker.update(1, sample(10), 0)
        for i in range(1, 301):
            tracker.update(1, sample(10+i*.1, math.sin(i*.1), math.cos(i*.1), i*.1), 0)
            self.assertTrue(np.isfinite(tracker.covariance).all())
            np.testing.assert_allclose(tracker.covariance, tracker.covariance.T, atol=1e-12)
            self.assertGreaterEqual(np.linalg.eigvalsh(tracker.covariance).min(), -1e-10)

    def test_local_tracker_does_not_modify_belief(self):
        arrays = [x.copy() for x in (self.motion.probability, self.motion.grid_s, self.motion.heading_offset)]
        tracker = self.tracker();tracker.update(1, sample(10), 0)
        tracker.update(1, sample(11, 1, 1, .2), 0)
        tracker.goal(2);tracker.belief_disagreement()
        for a, b in zip(arrays, (self.motion.probability, self.motion.grid_s, self.motion.heading_offset)):
            np.testing.assert_array_equal(a, b)

    def test_invalid_uncertainties_rejected(self):
        for values in [dict(yaw_std=-1), dict(translation_noise=float('nan'))]:
            with self.assertRaises(ValueError):
                LocalPoseConfig(**values)


if __name__ == '__main__':
    unittest.main()
