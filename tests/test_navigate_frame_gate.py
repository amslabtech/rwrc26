"""Merged camera gating, image-time localization and goal termination (no ROS master)."""
import json
from contextlib import contextmanager
from pathlib import Path
import sys
import tempfile
from threading import Condition, Event, Lock, Thread
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rospy
import torch
from nav_msgs.msg import Odometry

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "src/placenav")]

from placenav import navigate as navigation
from placenav.frame_gate import FrameGate
from placenav.goal_evaluation import GoalEvaluationConfig
from placenav.local_pose import LocalPoseTracker
from placenav.place_recognition.odometry_motion import OdometryHistory, OdometryMotionModel


class NavigateFrameGateTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="placenav-merge-")
        self.addCleanup(self.temp.cleanup)
        self.trace = []
        self.samples = []

    def make_node(self, closest=0):
        path = Path(self.temp.name) / "node_odometry.json"
        names = [str(i) + ".jpg" for i in range(3)]
        path.write_text(json.dumps(dict(
            schema_version=1, units=dict(position="m", angle="rad"),
            child_frame_id="base_link",
            nodes=[dict(id=i, image=name, pose=dict(x=i, y=0, yaw=0))
                   for i, name in enumerate(names)])))
        motion = OdometryMotionModel(path, names, noise=.1)
        node = navigation.PlaceNavNode.__new__(navigation.PlaceNavNode)
        node.motion_model = "odometry"
        node.local_pose_enabled = True
        node.goal_evaluation_config = GoalEvaluationConfig()
        node.robot_config = dict(frame_rate=5, max_v=1.)
        node.odometry_topic = "/test/odom"
        node.odom_history = OdometryHistory(.2, max_speed=3.)
        node.obs_lock, node.odom_lock = Lock(), Lock()
        node.odom_condition = Condition(node.odom_lock)
        node.frame_gate = FrameGate(5, min_period_fraction=0, is_shutdown=lambda: rospy.is_shutdown())
        node._last_localization_stamp = node._last_odom_epoch = None
        node.goal_selector = node.local_pose_tracker = None
        node._goal_reached = node._terminal_stop_sent = False
        node.context_size, node.context_queue = 0, []
        node.device = torch.device("cpu")
        node.map_size = 3
        node.topomap_images = [torch.zeros(1, 3, 2, 2) for _ in names]
        node.waypoint_scale = .1  # OmniVLA-edge's fixed XY scale.
        node._image_transform = node._vpr_transform = lambda image: (
            torch.from_numpy(image.copy()).permute(2, 0, 1).float())
        for name in ("waypoint_pub", "viz_pub", "goal_pub", "stop_pub",
                     "local_pose_pub", "goal_evaluation_pub"):
            setattr(node, name, Mock())

        def localize(image, odometry, initialize=False):
            self.trace.append(("localization", odometry.stamp_ns))
            self.samples.append(odometry)
            if initialize:
                prior = np.zeros(3)
                prior[closest] = 1.
                motion.initialize(prior)
            motion.predict(odometry)
            belief = motion.observe(np.ones(3))
            return int(np.argmax(belief)), float(belief.max())

        node.place_recognition_querier = SimpleNamespace(
            motion=motion, last_diagnostics=None,
            initialize_model=Mock(side_effect=lambda image, odom: localize(image, odom, True)),
            match=Mock(side_effect=localize),
            reset_odometry=Mock(side_effect=motion.reset_odometry))

        def infer(context, goal):
            self.trace.append(("inference", node._last_localization_stamp))
            return torch.ones(1, 1), torch.tensor([[[10., 5., 1., 0.]] * 8])
        node.waypoint_model = Mock(side_effect=infer)
        return node

    @staticmethod
    def add_odom(node, stamp_ns, x):
        message = Odometry()
        message.header.stamp = rospy.Time(*divmod(stamp_ns, 10**9))
        message.header.frame_id, message.child_frame_id = "odom", "base_link"
        message.pose.pose.position.x = x
        message.pose.pose.orientation.w = 1.
        node.odom_callback(message)

    @staticmethod
    def image(node, stamp_ns):
        message = navigation.bridge.cv2_to_imgmsg(np.zeros((2, 2, 3), np.uint8), "rgb8")
        message.header.stamp = rospy.Time(*divmod(stamp_ns, 10**9))
        node.obs_callback(message)

    @staticmethod
    def navigation_args(mode="arrival"):
        argv = ["navigate.py", "--robot", "whill", "--topomap-dir", "route",
                "--robot-config-path", "/unused/robots.yaml",
                "--model-config-path", "/unused/models.yaml",
                "--model-weight-dir", "/unused/weights", "--topomap-base-dir", "/unused/maps",
                "--wp-model", "omnivla_edge", "--subgoal-update", mode, "--local-pose"]
        with patch.object(sys, "argv", argv):
            return navigation.parser.parse_args()

    def run_navigation(self, node, actions, mode="arrival"):
        args = self.navigation_args(mode)
        shutdown = [False]
        pending = iter(actions)
        gate = FrameGate(5, min_period_fraction=0, is_shutdown=lambda: shutdown[0])

        def next_frame():
            action = next(pending, None)
            if action is None:
                shutdown[0] = True
                gate.notify()
            else:
                action()
            gate.sleep()

        node.frame_gate = SimpleNamespace(notify=gate.notify, sleep=next_frame)
        selector_update = navigation.ProgressSubgoalSelector.update
        pose_update = LocalPoseTracker.update

        def select(selector, closest, stamp_ns):
            self.trace.append(("subgoal", stamp_ns))
            return selector_update(selector, closest, stamp_ns)

        def track(tracker, closest, odometry, epoch):
            self.trace.append(("local_pose", odometry.stamp_ns))
            return pose_update(tracker, closest, odometry, epoch)

        with patch.object(rospy, "is_shutdown", side_effect=lambda: shutdown[0]), \
                patch.object(rospy, "loginfo"), patch.object(rospy, "logwarn_throttle"), \
                patch.object(navigation.ProgressSubgoalSelector, "update", select), \
                patch.object(LocalPoseTracker, "update", track):
            node.navigate(args)

    def test_new_images_update_once_after_image_time_odom_in_original_order(self):
        for mode in ("nearest", "passage", "arrival"):
            with self.subTest(mode=mode):
                self.trace.clear()
                self.samples.clear()
                node = self.make_node()
                first, second, third = 10_050_000_000, 10_150_000_000, 10_250_000_000

                def first_frame():
                    self.add_odom(node, 10_000_000_000, 0.)
                    self.add_odom(node, 10_100_000_000, .02)
                    self.image(node, first)

                def delayed_odom():
                    self.add_odom(node, 10_200_000_000, .04)
                    self.image(node, second)

                def third_frame():
                    self.add_odom(node, 10_300_000_000, .06)
                    self.image(node, third)

                self.run_navigation(node, [
                    first_frame, lambda: self.image(node, first),
                    delayed_odom, lambda: self.image(node, second), third_frame], mode)
                querier = node.place_recognition_querier
                self.assertEqual(querier.initialize_model.call_count, 1)
                self.assertEqual(querier.match.call_count, 2)
                self.assertEqual([s.stamp_ns for s in self.samples], [first, second, third])
                np.testing.assert_allclose([s.pose[0] for s in self.samples], [.01, .03, .05])
                self.assertEqual(self.trace, [
                    (stage, stamp) for stamp in (first, second, third)
                    for stage in ("localization", "subgoal", "local_pose", "inference")])
                for publisher, timestamp in (
                        (node.goal_evaluation_pub, lambda m: m.query_timestamp),
                        (node.local_pose_pub, lambda m: m.header.stamp),
                        (node.viz_pub, lambda m: m.query_timestamp)):
                    self.assertEqual([timestamp(c.args[0]).to_nsec()
                                      for c in publisher.publish.call_args_list],
                                     [first, second, third])
                self.assertEqual(node.waypoint_pub.publish.call_count, 3)
                for call in node.waypoint_pub.publish.call_args_list:
                    np.testing.assert_allclose(call.args[0].data, [1., .5, 1., 0.])
                self.assertFalse(node._goal_reached)

    @contextmanager
    def run_async(self, node, function):
        shutdown, finished = Event(), Event()
        result, errors = [], []

        def run():
            try:
                result.append(function())
            except BaseException as error:
                errors.append(error)
            finally:
                finished.set()

        with patch.object(rospy, "is_shutdown", side_effect=shutdown.is_set), \
                patch.object(rospy, "loginfo"), patch.object(rospy, "logwarn"), \
                patch.object(rospy, "logwarn_throttle"):
            thread = Thread(target=run, daemon=True)
            thread.start()
            try:
                yield finished, result, shutdown
            finally:
                shutdown.set()
                with node.odom_condition:
                    node.odom_condition.notify_all()
                node.frame_gate.notify()
                thread.join(2)
                self.assertFalse(thread.is_alive(), "Navigation wait did not stop")
                self.assertEqual(errors, [])

    @staticmethod
    def watch_missing_odom(node):
        missing = Event()
        original = node.odom_history.at

        def at(stamp_ns):
            sample = original(stamp_ns)
            if sample is None:
                missing.set()
            return sample

        node.odom_history.at = at
        return missing

    def test_late_odom_processes_retained_context_without_republishing_image(self):
        node = self.make_node()
        node.context_size = 5  # OmniVLA-edge uses six images.
        node.frame_gate = FrameGate(5, is_shutdown=lambda: rospy.is_shutdown())
        first, second = 10_050_000_000, 10_150_000_000
        self.add_odom(node, 10_000_000_000, 0.)
        for i in range(6):
            self.image(node, first - (5-i)*50_000_000)
        node.frame_gate.sleep()  # Consume the initial camera notification.
        missing = self.watch_missing_odom(node)
        inferred = [Event(), Event()]
        original = node.waypoint_model.side_effect

        def infer(context, goal):
            output = original(context, goal)
            inferred[len(self.samples)-1].set()
            return output

        node.waypoint_model.side_effect = infer
        args = self.navigation_args()
        with self.run_async(node, lambda: node.navigate(args)):
            self.assertTrue(missing.wait(1), "The image never waited for odometry")
            self.image(node, second)  # Must not replace the context already waiting.
            self.add_odom(node, 10_100_000_000, .02)
            self.assertTrue(inferred[0].wait(1), "Odom arrival did not release the pending image")
            self.assertEqual(self.samples[0].stamp_ns, first)
            self.add_odom(node, 10_200_000_000, .04)
            self.assertTrue(inferred[1].wait(1))
        self.assertEqual([sample.stamp_ns for sample in self.samples], [first, second])
        np.testing.assert_allclose([sample.pose[0] for sample in self.samples], [.01, .03])
        self.assertEqual(node.place_recognition_querier.initialize_model.call_count, 1)
        self.assertEqual(node.place_recognition_querier.match.call_count, 1)

    def test_unfillable_odometry_gap_is_not_waited_or_extrapolated(self):
        node = self.make_node()
        self.add_odom(node, 10_000_000_000, 0.)
        self.add_odom(node, 11_000_000_000, 0.)
        with patch.object(rospy, "is_shutdown", return_value=False), \
                patch.object(node.odom_condition, "wait", side_effect=AssertionError("Unfillable gap waited")):
            self.assertIsNone(node._image_odometry(10_050_000_000))
        node.place_recognition_querier.initialize_model.assert_not_called()

    def test_odometry_epoch_change_discards_waiting_image(self):
        node = self.make_node()
        stamp = 10_050_000_000
        self.add_odom(node, 10_000_000_000, 0.)
        self.image(node, stamp)
        missing = self.watch_missing_odom(node)
        with self.run_async(node, lambda: node._image_odometry(stamp)) as (finished, result, _):
            self.assertTrue(missing.wait(1))
            self.add_odom(node, 9_000_000_000, 0.)
            self.assertTrue(finished.wait(1))
            self.assertEqual(result, [None])
        self.assertEqual(node.odom_history.epoch, 1)

    def test_epoch_reset_does_not_retry_discarded_image_from_an_old_notification(self):
        node = self.make_node()
        stamp = 10_050_000_000
        self.add_odom(node, 10_000_000_000, 0.)
        self.image(node, stamp)  # Leave its notification pending.
        missing = self.watch_missing_odom(node)
        inferred = Event()
        original = node.waypoint_model.side_effect

        def infer(context, goal):
            output = original(context, goal)
            inferred.set()
            return output

        node.waypoint_model.side_effect = infer
        args = self.navigation_args()
        with self.run_async(node, lambda: node.navigate(args)):
            self.assertTrue(missing.wait(1))
            self.add_odom(node, 9_000_000_000, 0.)  # Invalidate the pending image.
            self.add_odom(node, 9_950_000_000, 0.)
            self.add_odom(node, 10_100_000_000, 0.)
            self.assertFalse(inferred.wait(.3), "Discarded image was retried in the new odom epoch")
            self.image(node, 10_100_000_000)
            self.assertTrue(inferred.wait(1))
        self.assertEqual([sample.stamp_ns for sample in self.samples], [10_100_000_000])

    def test_camera_time_rewind_discards_waiting_image(self):
        node = self.make_node()
        stamp = 10_050_000_000
        self.add_odom(node, 10_000_000_000, 0.)
        self.image(node, stamp)
        missing = self.watch_missing_odom(node)
        with self.run_async(node, lambda: node._image_odometry(stamp)) as (finished, result, _):
            self.assertTrue(missing.wait(1))
            self.image(node, 9_050_000_000)
            self.assertTrue(finished.wait(1))
            self.assertEqual(result, [None])

    def test_odometry_wait_stops_without_more_sensor_messages(self):
        for terminal in (False, True):
            with self.subTest(terminal=terminal):
                node = self.make_node()
                stamp = 10_050_000_000
                self.add_odom(node, 10_000_000_000, 0.)
                self.image(node, stamp)
                missing = self.watch_missing_odom(node)
                with self.run_async(node, lambda: node._image_odometry(stamp)) as (finished, result, shutdown):
                    self.assertTrue(missing.wait(1))
                    if terminal:
                        node._terminal_stop_sent = True
                    else:
                        shutdown.set()
                    self.assertTrue(finished.wait(1))
                    self.assertEqual(result, [None])
                node.waypoint_model.assert_not_called()

    def test_final_arrival_requires_distinct_images_and_keeps_terminal_stop(self):
        node = self.make_node(closest=2)
        first, second = 10_050_000_000, 10_150_000_000

        def first_frame():
            self.add_odom(node, 10_000_000_000, 0.)
            self.add_odom(node, 10_100_000_000, 0.)
            self.image(node, first)

        def second_frame():
            self.add_odom(node, 10_200_000_000, 0.)
            self.image(node, second)

        self.run_navigation(node, [first_frame, lambda: self.image(node, first), second_frame])
        self.assertTrue(node._goal_reached and node._terminal_stop_sent)
        self.assertEqual(node.place_recognition_querier.initialize_model.call_count, 1)
        self.assertEqual(node.place_recognition_querier.match.call_count, 1)
        self.assertEqual(node.waypoint_model.call_count, 1)
        self.assertEqual(node.goal_selector.status, "GOAL_REACHED")
        self.assertEqual(len(node.viz_pub.publish.call_args_list[-1].args[0].waypoints), 0)
        for call in node.waypoint_pub.publish.call_args_list[-3:]:
            np.testing.assert_array_equal(call.args[0].data, [0., 0.])
        self.assertTrue(all(c.args[0].data for c in node.goal_pub.publish.call_args_list[-3:]))
        before = node.waypoint_pub.publish.call_count
        node.publish_terminal_stop()
        self.assertEqual(node.waypoint_pub.publish.call_count, before)


if __name__ == "__main__":
    unittest.main()
