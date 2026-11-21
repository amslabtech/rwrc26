"""Goal pose transfer and command freshness for --nav-role localization/waypoint."""
import math
from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from placenav.split_navigation import (SubgoalCommand, command_status, compose, move_goal_pose,
                                       valid_goal_pose)
from placenav.topomap_odometry import relative_pose


def command(stamp_s, subgoal=3, reached=False, pose=(2., 0., 0.)):
    return SubgoalCommand(round(stamp_s*1e9), subgoal-1, subgoal, reached, pose)


class MoveGoalPoseTest(unittest.TestCase):
    def assertPose(self, actual, expected):
        for a, e in zip(actual, expected):
            self.assertAlmostEqual(a, e, places=9)

    def test_forward_motion_brings_the_goal_closer(self):
        self.assertPose(move_goal_pose((2., .5, .1), (10., 5., 0.), (10.5, 5., 0.)), (1.5, .5, .1))

    def test_rotation_in_place_turns_the_goal(self):
        # Turning left by 90 deg: a goal straight ahead is now on the right, facing right.
        self.assertPose(move_goal_pose((2., 0., 0.), (0., 0., 0.), (0., 0., math.pi/2)),
                        (0., -2., -math.pi/2))

    def test_matches_the_relative_pose_of_a_fixed_target(self):
        rng = random.Random(0)
        for _ in range(100):
            target, then, now = ((rng.uniform(-20, 20), rng.uniform(-20, 20), rng.uniform(-math.pi, math.pi))
                                 for _ in range(3))
            moved = move_goal_pose(relative_pose(then, target), then, now)
            expected = relative_pose(now, target)
            self.assertPose(moved[:2], expected[:2])
            self.assertAlmostEqual(math.sin(moved[2] - expected[2]), 0., places=9)
            self.assertLessEqual(abs(moved[2]), math.pi)

    def test_compose_inverts_relative_pose(self):
        base, other = (1., -2., 2.5), (-3., 4., -2.9)
        self.assertPose(compose(base, relative_pose(base, other)), other)

    def test_same_time_keeps_the_goal(self):
        self.assertPose(move_goal_pose((1., 2., .3), (4., 5., 1.), (4., 5., 1.)), (1., 2., .3))


class CommandStatusTest(unittest.TestCase):
    @staticmethod
    def status(cmd, frame_s, previous_s=None, map_size=5):
        previous = None if previous_s is None else round(previous_s*1e9)
        return command_status(cmd, round(frame_s*1e9), previous, map_size, int(.6e9))

    def test_fresh_command_in_either_direction_is_used(self):
        for frame_s, previous_s in ((10.2, 10.0), (9.8, 9.6), (10.6, None), (10.6, 10.4)):
            self.assertIsNone(self.status(command(10.), frame_s, previous_s), frame_s)

    def test_first_frame_after_a_camera_gap_keeps_the_last_command(self):
        # Frames at 10.0 and, after a 2 s gap, 12.0: the 12.0 command cannot exist yet.
        self.assertIsNone(self.status(command(10.), 12., 10.))
        # The next frame needs a command newer than the previous (12.0) frame minus the timeout.
        self.assertIn('older than the previous camera frame', self.status(command(10.), 12.2, 12.))

    def test_stale_missing_or_invalid_commands_are_refused(self):
        self.assertIn('no subgoal command', self.status(None, 1.))
        self.assertIn('0.61 s older than the previous', self.status(command(10.), 10.81, 10.61))
        self.assertIn('older than the previous', self.status(command(10.), 10.61))
        self.assertIn('newer than the camera frame', self.status(command(10.), 9.3, 9.1))
        self.assertIn('outside the topomap', self.status(command(10., subgoal=5), 10.))

    def test_goal_pose_values_must_be_finite(self):
        self.assertEqual(valid_goal_pose((1, 2, 3)), (1., 2., 3.))
        self.assertIsNone(valid_goal_pose((1., float('nan'), 0.)))
        self.assertIsNone(valid_goal_pose((1., float('inf'), 0.)))


if __name__ == '__main__':
    unittest.main()
