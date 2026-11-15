"""Serialize the diagnostic track; no subscribers, TF or command publishers."""
import math

import rospy
from placenav_viz_msgs.msg import LocalGoalPose


def fill_pose(message, mean, covariance):
    message.pose.position.x, message.pose.position.y = map(float, mean[:2])
    message.pose.orientation.z = math.sin(mean[2] / 2)
    message.pose.orientation.w = math.cos(mean[2] / 2)
    # ROS order is x,y,z,roll,pitch,yaw. This is a planar model.
    for i, row in enumerate((0, 1, 5)):
        for j, col in enumerate((0, 1, 5)):
            message.covariance[6*row+col] = float(covariance[i, j])


def goal_pose_message(tracker, closest_node, subgoal, stamp, epoch):
    relative, covariance = tracker.goal(subgoal)
    message = LocalGoalPose()
    message.header.stamp = stamp
    message.header.frame_id = tracker.motion.child_frame_id
    message.current_node_idx, message.subgoal_idx = closest_node, subgoal
    message.anchor_node_idx = tracker.anchor_node
    message.anchor_timestamp = rospy.Time(*divmod(tracker.anchor_stamp_ns, 10**9))
    message.pose_timestamp = rospy.Time(*divmod(tracker.last_odometry.stamp_ns, 10**9))
    message.anchor_epoch, message.odometry_epoch = tracker.epoch, epoch
    message.diagnostic_only = message.heading_reference_assumed = True
    message.odometry_continuous, message.status = tracker.continuous, tracker.status
    message.anchor_node_probability = tracker.anchor_probability
    message.map_frame = 'placenav_topomap'
    fill_pose(message.robot_pose, tracker.mean, tracker.covariance)
    fill_pose(message.goal_pose, relative, covariance)
    message.belief_position_disagreement, message.belief_yaw_disagreement = tracker.belief_disagreement()
    return message
