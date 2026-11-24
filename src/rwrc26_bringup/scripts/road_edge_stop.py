#!/usr/bin/env python3
"""Interlock the final autonomous Twist stream on confirmed road edges."""
import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse


class RoadEdgeStop:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_observation = float("-inf")
        self.held = False
        self.detected = False
        self.timeout = float(rospy.get_param("~detection_timeout", 1.0))
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("detection_timeout must be positive and finite")
        self.output = rospy.Publisher("~output", Twist, queue_size=1)
        rospy.Subscriber("~input", Twist, self.command, queue_size=1)
        rospy.Subscriber("/road_edge/detected", Bool, self.observe, queue_size=1)
        rospy.Service("~reset", Trigger, self.reset)
        rospy.on_shutdown(self.stop)

    def fresh(self):
        return time.monotonic() - self.last_observation < self.timeout

    def observe(self, msg):
        with self.lock:
            self.last_observation = time.monotonic()
            self.detected = msg.data
            if self.detected:
                self.held = True
                self.output.publish(Twist())

    def command(self, msg):
        with self.lock:
            self.output.publish(msg if self.fresh() and not self.held else Twist())

    def reset(self, _request):
        with self.lock:
            if not self.fresh() or self.detected:
                return TriggerResponse(False, "Await a fresh clear observation")
            self.held = False
            return TriggerResponse(True, "Released; awaiting next planner command")

    def stop(self):
        with self.lock:
            self.held = True
            self.output.publish(Twist())

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.held or not self.fresh():
                    self.output.publish(Twist())
            time.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("road_edge_stop")
    RoadEdgeStop().run()
