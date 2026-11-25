#!/usr/bin/env python3
"""Interlock the final autonomous Twist stream on confirmed road edges."""
import json
import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse


class RoadEdgeStop:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_observation = float("-inf")
        self.held = False
        self.observed_target = None
        self.detected = False
        self.held_target = None
        self.released_targets = set()
        self.timeout = float(rospy.get_param("~detection_timeout", 1.0))
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("detection_timeout must be positive and finite")
        self.output = rospy.Publisher("~output", Twist, queue_size=1)
        rospy.Subscriber("~input", Twist, self.command, queue_size=1)
        if rospy.get_param("~once_per_target", False):
            rospy.Subscriber("/road_edge/gate_status", String, self.observe_target, queue_size=1)
        else:
            rospy.Subscriber("/road_edge/detected", Bool, self.observe, queue_size=1)
        rospy.Service("~reset", Trigger, self.reset)
        rospy.on_shutdown(self.stop)

    def fresh(self):
        return time.monotonic() - self.last_observation < self.timeout

    def observe(self, msg):
        self.record_detection(msg.data, None)

    def observe_target(self, msg):
        # Use detection and target from the same image, including on window transitions.
        try:
            status = json.loads(msg.data)
            detected = status["detected"]
            target = status["target"]
            if type(detected) is not bool or (detected and type(target) is not int):
                raise ValueError("expected a detection flag and a target node")
        except (KeyError, TypeError, ValueError) as exc:
            rospy.logwarn_throttle(2.0, "Invalid road edge gate status: %s", exc)
            return
        self.record_detection(detected, target)

    def record_detection(self, detected, target):
        with self.lock:
            self.last_observation = time.monotonic()
            self.observed_target = target
            self.detected = detected and target not in self.released_targets
            if self.detected:
                if not self.held:
                    self.held_target = target
                self.held = True
                self.output.publish(Twist())

    def command(self, msg):
        with self.lock:
            self.output.publish(msg if self.fresh() and not self.held else Twist())

    def reset(self, _request):
        with self.lock:
            if not self.fresh():
                return TriggerResponse(False, "Await a fresh observation")
            if self.detected and (self.held_target is None or self.observed_target != self.held_target):
                return TriggerResponse(False, "Current detection is not the held target")
            if self.held_target is not None:
                self.released_targets.add(self.held_target)
                self.held_target = None
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
