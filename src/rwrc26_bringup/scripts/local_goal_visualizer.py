#!/usr/bin/env python3
"""Display the relative image goal using live TF, without publishing commands."""
import copy
import math

import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped
from placenav_viz_msgs.msg import LocalGoalPose
from visualization_msgs.msg import Marker, MarkerArray


def clear_markers():
    return MarkerArray(markers=[Marker(action=Marker.DELETEALL)])


def valid_goal(message):
    pose = message.goal_pose.pose
    values = [pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    return (message.odometry_continuous and bool(message.header.frame_id) and
            message.header.stamp != rospy.Time() and
            message.pose_timestamp == message.header.stamp and
            all(math.isfinite(v) for v in values) and
            sum(v*v for v in values[3:]) > 1e-12)


def goal_markers(message, transform):
    """Use the body-relative goal, never the unregistered topomap robot pose."""
    if not valid_goal(message):
        return clear_markers()
    local = PoseStamped(header=copy.deepcopy(message.header), pose=copy.deepcopy(message.goal_pose.pose))
    q = local.pose.orientation
    norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
    q.x, q.y, q.z, q.w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    target = tf2_geometry_msgs.do_transform_pose(local, transform)
    header = copy.deepcopy(target.header)
    header.stamp = message.header.stamp
    point = target.pose.position
    origin = transform.transform.translation

    direction = Marker(header=header, ns='local_goal', id=0, type=Marker.ARROW, action=Marker.ADD)
    direction.pose.orientation.w = 1.
    direction.points = [Point(origin.x, origin.y, origin.z+.15), Point(point.x, point.y, point.z+.15)]
    direction.scale.x, direction.scale.y, direction.scale.z = .045, .14, .20
    direction.color.r, direction.color.g, direction.color.b, direction.color.a = .8, .2, 1., 1.

    heading = Marker(header=header, ns='local_goal', id=1, type=Marker.ARROW, action=Marker.ADD)
    heading.pose = copy.deepcopy(target.pose)
    heading.pose.position.z += .15
    heading.scale.x, heading.scale.y, heading.scale.z = .7, .12, .16
    heading.color.r, heading.color.g, heading.color.b, heading.color.a = 1., .5, .05, 1.

    label = Marker(header=header, ns='local_goal', id=2, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
    label.pose.position = copy.deepcopy(point)
    label.pose.position.z += .55
    label.pose.orientation.w = 1.
    label.scale.z = .22
    label.color.r = label.color.g = label.color.b = label.color.a = 1.
    distance = math.hypot(message.goal_pose.pose.position.x, message.goal_pose.pose.position.y)
    label.text = 'Goal {} | anchor {}\n{:.2f} m'.format(
        message.subgoal_idx, message.anchor_node_idx, distance)
    return MarkerArray(markers=[direction, heading, label])


class LocalGoalVisualizer:
    def __init__(self):
        self.target_frame = rospy.get_param('~target_frame', 'odom')
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.publisher = rospy.Publisher('/toponav/local_goal_markers', MarkerArray, queue_size=1, latch=True)
        self.subscriber = rospy.Subscriber('/toponav/local_goal_pose', LocalGoalPose, self.receive, queue_size=1)

    def receive(self, message):
        if not valid_goal(message):
            self.publisher.publish(clear_markers())
            return
        try:
            transform = self.buffer.lookup_transform(self.target_frame, message.header.frame_id,
                                                     message.header.stamp, rospy.Duration(.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            self.publisher.publish(clear_markers())
            rospy.logwarn_throttle(5, 'Cannot display local image goal: %s', error)
            return
        self.publisher.publish(goal_markers(message, transform))


if __name__ == '__main__':
    rospy.init_node('local_goal_visualizer')
    LocalGoalVisualizer()
    rospy.spin()
