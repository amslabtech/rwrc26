#!/usr/bin/env python3
"""Rotation-only heading measurement math (no LightGlue needed)."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from placenav.rotation_estimation import (LEVEL_UP, equidistant_bearings, kabsch,  # noqa: E402
                                          rotation_ransac, yaw_about)


def axis_rotation(axis, angle):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def scene(n=200, seed=0):
    rng = np.random.default_rng(seed)
    v = rng.normal(size=(n, 3))
    v[:, 2] = np.abs(v[:, 2]) + 0.3  # in front of the camera
    return v / np.linalg.norm(v, axis=1, keepdims=True)


class BearingTest(unittest.TestCase):
    def test_centre_is_the_optical_axis_and_angle_is_linear_in_radius(self):
        b = equidistant_bearings([[100, 50], [100 + 1175 * math.radians(30), 50]], 1175., 100, 50)
        np.testing.assert_allclose(b[0], [0, 0, 1], atol=1e-12)
        self.assertAlmostEqual(math.degrees(math.acos(b[1, 2])), 30, places=6)
        self.assertGreater(b[1, 0], 0)  # right in the image = +x


class RotationTest(unittest.TestCase):
    def test_left_turn_is_positive_and_moves_the_scene_right(self):
        live_turn = math.radians(40)  # live camera turned 40 deg left (CCW from above) of the node camera
        rotation = axis_rotation(LEVEL_UP, -live_turn)  # bearings rotate the opposite way
        self.assertAlmostEqual(math.degrees(yaw_about(rotation)), 40, places=6)
        self.assertGreater((rotation @ [0, 0, 1])[0], 0)

    def test_kabsch_recovers_rotation(self):
        a = scene(); r = axis_rotation([0.1, -1, 0.05], 0.7)
        np.testing.assert_allclose(kabsch(a, a @ r.T), r, atol=1e-9)

    def test_ransac_rejects_wrong_matches_and_parallax(self):
        for yaw_deg in (-120, -35, 0, 15, 80):
            with self.subTest(yaw=yaw_deg):
                a = scene(300, seed=yaw_deg + 200)
                r = axis_rotation(LEVEL_UP, -math.radians(yaw_deg)) @ axis_rotation([1, 0, 0], math.radians(3))
                b = a @ r.T
                rng = np.random.default_rng(1)
                bad = rng.choice(len(a), 90, replace=False)  # 30 % wrong matches
                b[bad] = scene(90, seed=99)
                b[:20] = b[:20] + np.array([0.08, 0, 0])  # near points with parallax
                b /= np.linalg.norm(b, axis=1, keepdims=True)
                fit = rotation_ransac(a, b, threshold_deg=1.0, iterations=500)
                self.assertIsNotNone(fit)
                rotation, inliers = fit
                self.assertAlmostEqual(math.degrees(yaw_about(rotation)), yaw_deg, delta=0.2)
                self.assertFalse(inliers[bad].any())

    def test_too_few_matches(self):
        self.assertIsNone(rotation_ransac(scene(2), scene(2)))


if __name__ == "__main__":
    unittest.main()
