#!/usr/bin/env python3
"""Heading alignment gate: image-driven rotation, hysteresis, fallback and latching."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from placenav.heading_align import HeadingAlignConfig, HeadingAligner  # noqa: E402
from placenav.heading_correction import HeadingEstimate  # noqa: E402

D = math.radians
YAWS = [D(a) for a in (0, 0, 30, 60, 90, 90)]


class Robot:
    """Measured dpsi = robot yaw - node yaw (+ error), or None."""
    def __init__(self, aligner, yaw=0.):
        self.aligner, self.yaw, self.t, self.visible, self.error = aligner, yaw, 0., True, 0.

    def measure(self, node):
        if not self.visible:
            return None
        return HeadingEstimate(dpsi=math.atan2(math.sin(self.yaw - YAWS[node] + self.error),
                                               math.cos(self.yaw - YAWS[node] + self.error)), std=0.05, support=99)

    def cycle(self, subgoal, arrived=True):
        self.t += 0.25
        self.aligner.new_cycle(self.measure, self.yaw, self.t)
        allowed = self.aligner.gate(subgoal) if arrived else None
        command = self.aligner.finish_cycle(subgoal + 1 if allowed else subgoal)
        if command.active:  # TEB stops within ~11 deg of the target
            remaining = math.atan2(math.sin(command.target_yaw - self.yaw), math.cos(command.target_yaw - self.yaw))
            self.yaw += remaining - math.copysign(min(abs(remaining), D(10)), remaining)
        return allowed, command


class HeadingAlignTest(unittest.TestCase):
    def setUp(self):
        self.aligner = HeadingAligner(YAWS, HeadingAlignConfig(start_deg=25, done_deg=15, timeout=20,
                                                               fallback_after=3))
        self.robot = Robot(self.aligner)

    def test_small_difference_advances_in_the_same_cycle(self):
        allowed, command = self.robot.cycle(1)
        self.assertTrue(allowed)
        self.assertFalse(command.active)

    def test_turn_on_the_spot_is_done_node_by_node(self):
        self.robot.yaw = 0.
        self.assertTrue(self.robot.cycle(1)[0])
        for node in (2, 3, 4):  # 30 deg steps at one position
            allowed, command = self.robot.cycle(node)
            self.assertFalse(allowed)
            self.assertTrue(command.active)
            self.assertEqual(command.source, 'image')
            self.assertAlmostEqual(command.target_yaw, YAWS[node])
            allowed, command = self.robot.cycle(node)
            self.assertTrue(allowed, node)
            self.assertLessEqual(abs(self.robot.yaw - YAWS[node]), D(15))
        self.assertAlmostEqual(self.robot.yaw, D(80))

    def test_hysteresis_keeps_rotating_between_done_and_start(self):
        self.robot.yaw = D(-40)
        self.assertFalse(self.robot.cycle(1)[0])
        self.robot.yaw = D(-20)  # below start, above done: not finished
        allowed, command = self.robot.cycle(1)
        self.assertFalse(allowed)
        self.assertTrue(command.active)

    def test_rotation_continues_when_the_arrival_condition_flickers(self):
        self.robot.yaw = D(-60)
        self.robot.cycle(1)
        self.robot.yaw = D(-60)  # not turned yet in this cycle
        allowed, command = self.robot.cycle(1, arrived=False)
        self.assertIsNone(allowed)
        self.assertTrue(command.active)

    def test_wrong_way_heading_turns_back(self):
        self.robot.yaw = D(-50)
        _, command = self.robot.cycle(2)
        self.assertAlmostEqual(command.target_yaw, YAWS[2])

    def test_no_image_holds_then_rotates_by_the_taught_step(self):
        self.robot.visible = False
        for _ in range(2):
            allowed, command = self.robot.cycle(3)
            self.assertFalse(allowed)
            self.assertEqual(command.status, 'WAITING_IMAGE')
            self.assertTrue(command.active)
            self.assertAlmostEqual(command.target_yaw, 0.)  # hold
        _, command = self.robot.cycle(3)
        self.assertEqual((command.status, command.source), ('ROTATING_ODOM', 'odometry'))
        self.assertAlmostEqual(command.target_yaw, D(30))  # node 2 -> 3 step
        self.robot.yaw = D(25)
        self.assertTrue(self.robot.cycle(3)[0])

    def test_no_image_on_a_straight_advances_after_the_wait(self):
        self.robot.visible = False
        for _ in range(2):
            self.assertFalse(self.robot.cycle(5)[0])
        self.assertTrue(self.robot.cycle(5)[0])  # taught step 0

    def test_timeout_lets_the_subgoal_advance(self):
        self.robot.yaw = D(-90)
        self.robot.error = D(40)  # the measurement never converges
        self.aligner.new_cycle(self.robot.measure, self.robot.yaw, 0.)
        self.assertFalse(self.aligner.gate(1))
        self.aligner.finish_cycle(1)
        self.aligner.new_cycle(self.robot.measure, self.robot.yaw, 25.)
        self.assertTrue(self.aligner.gate(1))
        command = self.aligner.finish_cycle(2)
        self.assertEqual((command.status, command.active, command.node), ('TIMEOUT', False, 1))
        self.aligner.new_cycle(self.robot.measure, self.robot.yaw, 26.)
        self.assertEqual(self.aligner.finish_cycle(2).status, 'IDLE')

    def test_alignment_in_the_arrival_cycle_is_reported_with_its_measurement(self):
        self.robot.yaw = D(10)
        allowed, command = self.robot.cycle(1)
        self.assertTrue(allowed)
        self.assertEqual((command.status, command.active, command.node, command.source), ('ALIGNED', False, 1, 'image'))
        self.assertAlmostEqual(command.dpsi, D(10))

    def test_subgoal_change_and_final_goal_reset(self):
        self.robot.yaw = D(-60)
        self.robot.cycle(1)
        self.aligner.new_cycle(self.robot.measure, self.robot.yaw, 1.)
        self.assertFalse(self.aligner.finish_cycle(3).active)  # recovery moved the subgoal
        self.robot.cycle(1)
        self.assertFalse(self.aligner.finish_cycle(1, reached_goal=True).active)

    def test_config_validation(self):
        with self.assertRaises(ValueError):
            HeadingAlignConfig(start_deg=10, done_deg=15)


if __name__ == "__main__":
    unittest.main()
