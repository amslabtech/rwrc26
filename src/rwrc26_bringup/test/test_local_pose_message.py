"""The relative pose must have the image's ID, timestamp and body frame."""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

import numpy as np
import rospy

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from placenav.local_pose_ros import goal_pose_message
from placenav.topomap_odometry import OdomSample


class LocalPoseMessageTest(unittest.TestCase):
    def test_frame_covariance_metadata_and_invalid_track_time(self):
        covariance = np.array([[1, .2, .3], [.2, 2, .4], [.3, .4, 3.]])
        tracker = SimpleNamespace(
            goal=lambda g: (np.array([g, -1., .5]), covariance),
            motion=SimpleNamespace(child_frame_id='test_base'),
            mean=np.array([10., 20., .3]), covariance=covariance,
            anchor_node=2, anchor_stamp_ns=10_000_000_123,
            last_odometry=OdomSample(11_000_000_456, (0, 0, 0), 'odom', 'test_base'),
            epoch=3, continuous=False, status='ODOMETRY_DISCONTINUITY',
            anchor_probability=.2, belief_disagreement=lambda: (4., .6))
        message = goal_pose_message(tracker, 8, 9, rospy.Time(12, 789), 4)
        self.assertEqual(message.header.stamp.to_nsec(), 12_000_000_789)
        self.assertEqual(message.pose_timestamp.to_nsec(), 11_000_000_456)
        self.assertEqual(message.header.frame_id, 'test_base')
        self.assertEqual(message.anchor_timestamp.to_nsec(), 10_000_000_123)
        self.assertEqual(message.subgoal_idx, 9)
        self.assertEqual(message.goal_pose.pose.position.x, 9)
        self.assertEqual(message.robot_pose.pose.position.x, 10)
        self.assertFalse(message.odometry_continuous)
        self.assertTrue(message.diagnostic_only and message.heading_reference_assumed)
        for msg in [message.goal_pose, message.robot_pose]:
            actual = np.array(msg.covariance).reshape(6, 6)[np.ix_([0, 1, 5], [0, 1, 5])]
            np.testing.assert_array_equal(actual, covariance)
        self.assertEqual(message.anchor_epoch, 3)
        self.assertEqual(message.odometry_epoch, 4)


if __name__ == '__main__':
    unittest.main()
