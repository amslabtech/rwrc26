import importlib.util
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist


spec = importlib.util.spec_from_file_location(
    "road_edge_stop", Path(__file__).resolve().parents[1] / "scripts/road_edge_stop.py")
stop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stop)


class RoadEdgeStopTest(unittest.TestCase):
    def setUp(self):
        ros = patch.object(stop, "rospy")
        self.ros = ros.start()
        self.addCleanup(ros.stop)
        self.ros.get_param.side_effect = lambda name, default: default
        self.node = stop.RoadEdgeStop()
        self.move = Twist()
        self.move.linear.x = .2

    def observe(self, target, detected):
        self.node.observe_target(String(data=json.dumps(dict(target=target, detected=detected))))

    def output_speed(self):
        self.node.command(self.move)
        return self.node.output.publish.call_args[0][0].linear.x

    def test_release_applies_only_to_target_that_stopped(self):
        self.observe(1, True)
        self.assertEqual(self.output_speed(), 0.)
        self.observe(1, True)
        self.assertEqual(self.output_speed(), 0.)
        self.assertTrue(self.node.reset(None).success)
        self.observe(1, True)
        self.assertEqual(self.output_speed(), .2)
        self.observe(2, True)
        self.assertEqual(self.output_speed(), 0.)
        self.observe(2, False)
        self.assertTrue(self.node.reset(None).success)
        for target in (1, 2):
            self.observe(target, True)
            self.assertEqual(self.output_speed(), .2)
        self.node.last_observation = float("-inf")
        self.assertEqual(self.output_speed(), 0.)
        self.assertFalse(self.node.reset(None).success)

    def test_different_active_target_cannot_be_released_by_mistake(self):
        self.observe(1, True)
        self.observe(2, True)
        self.assertFalse(self.node.reset(None).success)
        self.assertEqual(self.output_speed(), 0.)
        self.observe(None, False)
        self.assertTrue(self.node.reset(None).success)
        self.observe(2, True)
        self.assertEqual(self.output_speed(), 0.)

    def test_reset_without_stop_does_not_consume_target(self):
        self.observe(1, False)
        self.assertTrue(self.node.reset(None).success)
        self.observe(1, True)
        self.assertEqual(self.output_speed(), 0.)

    def test_ungated_detections_can_stop_again(self):
        self.node.observe(Bool(data=True))
        self.assertFalse(self.node.reset(None).success)
        self.node.observe(Bool(data=False))
        self.assertTrue(self.node.reset(None).success)
        self.node.observe(Bool(data=True))
        self.assertEqual(self.output_speed(), 0.)


if __name__ == "__main__":
    unittest.main()
