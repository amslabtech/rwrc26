#!/usr/bin/env python3

import argparse
import math
from typing import Iterable, List, Sequence, Tuple

import rospy
import tf2_geometry_msgs  # noqa: F401; registers geometry message conversions
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Path
from placenav_viz_msgs.msg import Viz


XY = Tuple[float, float]


def waypoint_yaws(points: Sequence[XY]) -> List[float]:
    """Return a forward-facing yaw for every point in a local path."""
    if not points:
        return []
    if len(points) == 1:
        return [0.0]

    yaws = []
    previous_yaw = 0.0
    for index, point in enumerate(points):
        if index + 1 < len(points):
            next_point = points[index + 1]
            dx = next_point[0] - point[0]
            dy = next_point[1] - point[1]
        else:
            previous_point = points[index - 1]
            dx = point[0] - previous_point[0]
            dy = point[1] - previous_point[1]

        if math.hypot(dx, dy) > 1e-9:
            previous_yaw = math.atan2(dy, dx)
        yaws.append(previous_yaw)
    return yaws


def quaternion_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def local_poses(
    waypoints: Iterable[Point],
    stamp: rospy.Time,
    frame_id: str,
    include_origin: bool,
    x_offset: float = 0.0,
) -> List[PoseStamped]:
    # Model waypoints are relative to the camera; x_offset moves them from the
    # camera to frame_id. The inserted origin stays at frame_id (the robot).
    points = [(point.x + x_offset, point.y) for point in waypoints]
    if include_origin:
        points.insert(0, (0.0, 0.0))

    yaws = waypoint_yaws(points)
    poses = []
    for (x, y), yaw in zip(points, yaws):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = quaternion_from_yaw(yaw)
        poses.append(pose)
    return poses


def rotation_poses(x: float, y: float, yaw: float, target_yaw: float,
                   stamp: rospy.Time, frame_id: str) -> List[PoseStamped]:
    """An in-place rotation path: the robot pose, then the same position at target_yaw."""
    poses = []
    for heading in (yaw, target_yaw):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = quaternion_from_yaw(heading)
        poses.append(pose)
    return poses


class GnmPathAdapter:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/toponav/viz_info")
        self.output_topic = rospy.get_param("~output_topic", "/placenav/gnm_path")
        self.source_frame = rospy.get_param("~source_frame", "base_link")
        self.target_frame = rospy.get_param("~target_frame", "odom")
        self.include_origin = rospy.get_param("~include_origin", True)
        self.expected_waypoints = rospy.get_param("~expected_waypoints", 5)
        self.waypoint_x_offset = float(rospy.get_param("~waypoint_x_offset", 0.0))
        self.transform_timeout = rospy.Duration(
            rospy.get_param("~transform_timeout", 0.2)
        )

        # navigate.py --heading-align: while active, rotate in place instead of following waypoints.
        self.heading_align_topic = rospy.get_param("~heading_align_topic", "/toponav/heading_align")
        self.heading_align_timeout = rospy.Duration(rospy.get_param("~heading_align_timeout", 1.0))
        self.heading_align = None
        self.heading_align_received = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.path_publisher = rospy.Publisher(self.output_topic, Path, queue_size=1)
        self.viz_subscriber = rospy.Subscriber(
            self.input_topic, Viz, self.viz_callback, queue_size=1
        )

        self.heading_align_subscriber = None
        if self.heading_align_topic:
            try:
                from placenav_viz_msgs.msg import HeadingAlign
            except ImportError:
                rospy.logwarn("placenav_viz_msgs has no HeadingAlign (rebuild it); heading alignment is ignored")
            else:
                self.heading_align_subscriber = rospy.Subscriber(
                    self.heading_align_topic, HeadingAlign, self.heading_align_callback, queue_size=1
                )

        rospy.loginfo(
            "GNM path adapter: %s (%s, waypoint x offset %.3f m) -> %s (%s)",
            self.input_topic,
            self.source_frame,
            self.waypoint_x_offset,
            self.output_topic,
            self.target_frame,
        )

    def heading_align_active(self) -> bool:
        return (
            self.heading_align is not None
            and self.heading_align.active
            and rospy.Time.now() - self.heading_align_received <= self.heading_align_timeout
        )

    def heading_align_callback(self, message):
        was_active = self.heading_align_active()
        self.heading_align, self.heading_align_received = message, rospy.Time.now()
        if message.active != was_active:
            rospy.loginfo(
                "Heading align %s: %s at node %d",
                "start" if message.active else "end", message.status, message.node_idx,
            )
        if not message.active:
            return
        if message.header.frame_id != self.target_frame:
            rospy.logwarn_throttle(
                2.0, "Heading align pose frame %s is not %s; not rotating",
                message.header.frame_id, self.target_frame,
            )
            return
        path = Path()
        path.header.stamp = message.header.stamp
        path.header.frame_id = self.target_frame
        path.poses = rotation_poses(
            message.robot.x, message.robot.y, message.robot.theta, message.target_yaw,
            message.header.stamp, self.target_frame,
        )
        self.path_publisher.publish(path)

    def viz_callback(self, message: Viz):
        if self.heading_align_active():
            return  # the rotation path from heading_align_callback is in charge
        if not message.waypoints:
            rospy.logwarn_throttle(2.0, "GNM path contains no waypoints")
            return
        if len(message.waypoints) != self.expected_waypoints:
            rospy.logwarn_throttle(
                2.0,
                "Expected %d GNM waypoints but received %d",
                self.expected_waypoints,
                len(message.waypoints),
            )

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                message.query_timestamp,
                self.transform_timeout,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            rospy.logwarn_throttle(
                1.0,
                "Cannot transform GNM path from %s to %s at %.9f: %s",
                self.source_frame,
                self.target_frame,
                message.query_timestamp.to_sec(),
                error,
            )
            return

        path = Path()
        path.header.stamp = message.query_timestamp
        path.header.frame_id = self.target_frame
        path.poses = [
            tf2_geometry_msgs.do_transform_pose(pose, transform)
            for pose in local_poses(
                message.waypoints,
                message.query_timestamp,
                self.source_frame,
                self.include_origin,
                self.waypoint_x_offset,
            )
        ]
        for pose in path.poses:
            pose.header = path.header
        self.path_publisher.publish(path)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert PlaceNav GNM waypoints into an odom-frame nav_msgs/Path"
    )
    parser.add_argument("--input-topic", default="/toponav/viz_info")
    parser.add_argument("--output-topic", default="/placenav/gnm_path")
    parser.add_argument("--source-frame", default="base_link")
    parser.add_argument("--target-frame", default="odom")
    parser.add_argument("--transform-timeout", type=float, default=0.2)
    parser.add_argument("--expected-waypoints", type=int, default=5)
    parser.add_argument("--no-origin", action="store_true")
    parser.add_argument("--heading-align-topic", default="/toponav/heading_align",
                        help="navigate.py --heading-align output; empty disables")
    parser.add_argument("--heading-align-timeout", type=float, default=1.0)
    return parser.parse_args(rospy.myargv()[1:])


def main():
    rospy.init_node("gnm_path_adapter", anonymous=False)
    args = parse_args()
    rospy.set_param("~input_topic", args.input_topic)
    rospy.set_param("~output_topic", args.output_topic)
    rospy.set_param("~source_frame", args.source_frame)
    rospy.set_param("~target_frame", args.target_frame)
    rospy.set_param("~transform_timeout", args.transform_timeout)
    rospy.set_param("~expected_waypoints", args.expected_waypoints)
    rospy.set_param("~include_origin", not args.no_origin)
    rospy.set_param("~heading_align_topic", args.heading_align_topic)
    rospy.set_param("~heading_align_timeout", args.heading_align_timeout)
    GnmPathAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
