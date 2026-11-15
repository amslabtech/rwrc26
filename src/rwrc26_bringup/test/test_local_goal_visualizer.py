"""Goal markers must use live body TF at the image time and reject stale poses."""
import copy
import importlib.util
import math
from pathlib import Path
import unittest
from unittest import mock

import rospy
from geometry_msgs.msg import TransformStamped
from placenav_viz_msgs.msg import LocalGoalPose
from visualization_msgs.msg import Marker


spec = importlib.util.spec_from_file_location('local_goal_visualizer',
    Path(__file__).resolve().parents[1]/'scripts/local_goal_visualizer.py')
viz = importlib.util.module_from_spec(spec)
spec.loader.exec_module(viz)


class LocalGoalVisualizerTest(unittest.TestCase):
    def message(self):
        message = LocalGoalPose()
        message.header.stamp = message.pose_timestamp = rospy.Time(42, 123)
        message.header.frame_id = 'test_base'
        message.odometry_continuous = True
        message.subgoal_idx, message.anchor_node_idx = 12, 11
        message.status = 'TRACKING_POSITION_ANCHOR'
        message.goal_pose.pose.position.x, message.goal_pose.pose.position.y = 2., 1.
        message.goal_pose.pose.orientation.z = math.sin(.3/2)
        message.goal_pose.pose.orientation.w = math.cos(.3/2)
        # This virtual map pose must not be mistaken for the live odom pose.
        message.robot_pose.pose.position.x = 10000.
        return message

    def transform(self):
        transform = TransformStamped()
        transform.header.frame_id = 'test_odom'
        transform.child_frame_id = 'test_base'
        transform.header.stamp = rospy.Time(42, 123)
        transform.transform.translation.x, transform.transform.translation.y = 10., 20.
        transform.transform.rotation.z = math.sin(math.pi/4)
        transform.transform.rotation.w = math.cos(math.pi/4)
        return transform

    def test_live_transform_goal_heading_label_and_input_immutability(self):
        message = self.message()
        before = copy.deepcopy(message)
        markers = viz.goal_markers(message, self.transform()).markers
        direction, heading, label = markers
        self.assertEqual(message, before)
        self.assertEqual([m.id for m in markers], [0, 1, 2])
        for marker in markers:
            self.assertEqual(marker.header.frame_id, 'test_odom')
            self.assertEqual(marker.header.stamp, message.header.stamp)
            self.assertEqual(marker.action, Marker.ADD)
        self.assertAlmostEqual(direction.points[0].x, 10.)
        self.assertAlmostEqual(direction.points[0].y, 20.)
        self.assertAlmostEqual(direction.points[1].x, 9.)
        self.assertAlmostEqual(direction.points[1].y, 22.)
        self.assertAlmostEqual(heading.pose.position.x, 9.)
        self.assertAlmostEqual(heading.pose.position.y, 22.)
        self.assertAlmostEqual(2*math.atan2(heading.pose.orientation.z, heading.pose.orientation.w), math.pi/2+.3)
        self.assertIn('Goal 12 | anchor 11', label.text)
        self.assertIn('2.24 m', label.text)

    def test_invalid_or_stale_pose_clears_existing_display(self):
        for kind in ['invalid', 'stale', 'nan', 'zero_quaternion', 'empty_frame', 'zero_stamp']:
            message = self.message()
            if kind == 'invalid': message.odometry_continuous = False
            elif kind == 'stale': message.pose_timestamp = rospy.Time(41)
            elif kind == 'nan': message.goal_pose.pose.position.x = float('nan')
            elif kind == 'zero_quaternion': message.goal_pose.pose.orientation.z = message.goal_pose.pose.orientation.w = 0
            elif kind == 'empty_frame': message.header.frame_id = ''
            else: message.header.stamp = message.pose_timestamp = rospy.Time()
            self.assertEqual(viz.goal_markers(message, self.transform()).markers[0].action, Marker.DELETEALL)

    def test_callback_requests_image_time_tf_and_publishes_markers(self):
        node = object.__new__(viz.LocalGoalVisualizer)
        node.target_frame = 'test_odom'
        node.buffer, node.publisher = mock.Mock(), mock.Mock()
        node.buffer.lookup_transform.return_value = self.transform()
        message = self.message()
        node.receive(message)
        node.buffer.lookup_transform.assert_called_once_with('test_odom', 'test_base', message.header.stamp, rospy.Duration(.2))
        self.assertEqual(len(node.publisher.publish.call_args[0][0].markers), 3)

    def test_missing_tf_clears_instead_of_reusing_an_old_goal(self):
        node = object.__new__(viz.LocalGoalVisualizer)
        node.target_frame = 'test_odom'
        node.buffer, node.publisher = mock.Mock(), mock.Mock()
        node.buffer.lookup_transform.side_effect = viz.tf2_ros.LookupException('missing')
        with mock.patch.object(rospy, 'logwarn_throttle'):
            node.receive(self.message())
        self.assertEqual(node.publisher.publish.call_args[0][0].markers[0].action, Marker.DELETEALL)

    def test_invalid_track_does_not_look_up_tf(self):
        node = object.__new__(viz.LocalGoalVisualizer)
        node.buffer, node.publisher = mock.Mock(), mock.Mock()
        message = self.message(); message.odometry_continuous = False
        node.receive(message)
        node.buffer.lookup_transform.assert_not_called()
        self.assertEqual(node.publisher.publish.call_args[0][0].markers[0].action, Marker.DELETEALL)


if __name__ == '__main__':
    unittest.main()
