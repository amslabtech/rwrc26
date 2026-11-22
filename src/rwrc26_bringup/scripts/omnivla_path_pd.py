#!/usr/bin/env python3
"""Track a stamped base_link OmniVLA waypoint using longitudinal and heading PD."""
import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from placenav_viz_msgs.msg import Viz
from std_msgs.msg import Bool


class PathTracker:
    def __init__(self):
        self.index = rospy.get_param("~waypoint_index", 4)
        if isinstance(self.index, bool) or not isinstance(self.index, int) or self.index < 0:
            raise ValueError("waypoint_index must be a non-negative integer")
        self.timeout = self.parameter("path_timeout", 0.75, positive=True)
        self.max_v = self.parameter("max_v", 0.30, positive=True)
        self.max_w = self.parameter("max_w", 0.30, positive=True)
        self.kp_v = self.parameter("kp_v", 1.0)
        self.kd_v = self.parameter("kd_v", 0.05)
        self.kp_w = self.parameter("kp_w", 1.5)
        self.kd_w = self.parameter("kd_w", 0.05)
        self.lock = threading.Lock()
        self.stopped = False
        self.stamp = None
        self.received = None
        self.previous = None
        self.command = Twist()
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/toponav/viz_info", Viz, self.path, queue_size=1)
        rospy.Subscriber("/toponav/reached_goal", Bool, self.stop_signal, queue_size=1)
        rospy.Subscriber("/toponav/stop", Bool, self.stop_signal, queue_size=1)
        rospy.on_shutdown(self.stop)

    @staticmethod
    def parameter(name, default, positive=False):
        value = float(rospy.get_param("~" + name, default))
        if not math.isfinite(value) or value < 0 or (positive and value == 0):
            raise ValueError("invalid parameter: " + name)
        return value

    def clear(self):
        self.command = Twist()
        self.previous = None
        self.received = None

    def path(self, msg):
        with self.lock:
            if self.stopped:
                return
            stamp = msg.query_timestamp
            age = (rospy.Time.now() - stamp).to_sec()
            if (stamp.is_zero() or not 0 <= age <= self.timeout or
                    len(msg.waypoints) <= self.index or
                    any(not math.isfinite(v) for p in msg.waypoints for v in (p.x, p.y))):
                self.clear()
                self.pub.publish(self.command)
                return
            if self.stamp is not None and stamp <= self.stamp:
                return
            p = msg.waypoints[self.index]
            dt = (stamp - self.stamp).to_sec() if self.stamp is not None else 0
            self.stamp = stamp
            if p.x <= 0 and abs(p.y) < 1e-6:
                self.clear()
                self.pub.publish(self.command)
                return
            heading = math.atan2(p.y, p.x)
            dx = da = 0.0
            if (self.previous is not None and self.previous[2] == msg.subgoal_idx
                    and 0 < dt <= self.timeout):
                dx = (p.x - self.previous[0]) / dt
                angle = heading - self.previous[1]
                da = math.atan2(math.sin(angle), math.cos(angle)) / dt
            self.previous = (p.x, heading, msg.subgoal_idx)
            self.received = time.monotonic()
            self.command = Twist()
            if p.x > 0:
                self.command.linear.x = min(self.max_v, max(0, self.kp_v*p.x + self.kd_v*dx))
            self.command.angular.z = max(-self.max_w, min(self.max_w, self.kp_w*heading + self.kd_w*da))

    def stop_signal(self, msg):
        if msg.data:
            self.stop()

    def stop(self):
        with self.lock:
            self.stopped = True
            self.clear()
            self.pub.publish(self.command)

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.received is not None:
                    age = (rospy.Time.now() - self.stamp).to_sec()
                    if not 0 <= age <= self.timeout or time.monotonic()-self.received > self.timeout:
                        self.clear()
                self.pub.publish(self.command)
            time.sleep(0.05)


if __name__ == "__main__":
    rospy.init_node("omnivla_path_pd")
    PathTracker().run()
