#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from placenav.gnm_path_adapter import waypoint_yaws, local_poses, rotation_poses  # noqa: E402
from placenav.utils import waypoints_to_ros_msg
import numpy as np
import rospy


class WaypointYawsTest(unittest.TestCase):
    def test_nomad_xy_and_vint_xy_heading_to_path(self):
        for count, width in ((8, 2), (5, 4)):
            points = np.zeros((count, width))
            points[:, 0] = np.arange(1, count + 1) * 0.1
            messages = waypoints_to_ros_msg(points)
            poses = local_poses(messages, rospy.Time(12), "base_link", True)
            self.assertEqual(len(poses), count + 1)
            self.assertAlmostEqual(poses[-1].pose.position.x, count * 0.1)

    def test_x_offset_moves_waypoints_but_not_origin(self):
        points = np.zeros((2, 2))
        points[:, 0] = [0.5, 1.0]
        points[:, 1] = [0.0, 0.2]
        messages = waypoints_to_ros_msg(points)
        poses = local_poses(messages, rospy.Time(12), "base_link", True, 0.62)
        self.assertEqual(len(poses), 3)
        self.assertAlmostEqual(poses[0].pose.position.x, 0.0)
        self.assertAlmostEqual(poses[1].pose.position.x, 1.12)
        self.assertAlmostEqual(poses[2].pose.position.x, 1.62)
        self.assertAlmostEqual(poses[2].pose.position.y, 0.2)

    def test_empty_path(self):
        self.assertEqual(waypoint_yaws([]), [])

    def test_single_point_faces_forward(self):
        self.assertEqual(waypoint_yaws([(1.0, 0.0)]), [0.0])

    def test_straight_path_faces_forward(self):
        self.assertEqual(waypoint_yaws([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]), [0.0, 0.0, 0.0])

    def test_curve_uses_each_segment_and_repeats_last_heading(self):
        result = waypoint_yaws([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
        self.assertAlmostEqual(result[0], 0.0)
        self.assertAlmostEqual(result[1], math.pi / 2.0)
        self.assertAlmostEqual(result[2], math.pi / 2.0)


class RotationPosesTest(unittest.TestCase):
    def test_same_position_from_current_to_target_yaw(self):
        poses = rotation_poses(1.0, 2.0, 0.3, -1.2, rospy.Time(5), "odom")
        self.assertEqual(len(poses), 2)  # the TEB bridge needs at least two poses
        for pose in poses:
            self.assertEqual(pose.header.frame_id, "odom")
            self.assertEqual((pose.pose.position.x, pose.pose.position.y), (1.0, 2.0))
        yaws = [2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w) for p in poses]
        self.assertAlmostEqual(yaws[0], 0.3)
        self.assertAlmostEqual(yaws[1], -1.2)


if __name__ == "__main__":
    unittest.main()
