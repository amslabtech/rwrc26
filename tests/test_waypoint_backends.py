import math
import sys
import unittest
from pathlib import Path

import torch
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from placenav.waypoint_backends import (
    decode_actions, load_waypoint_backend, NoMaDAdapter)

ROOT = Path(__file__).resolve().parents[1]
OMNIVLA_SOURCES = ROOT / "src/model_weights/waypoint/sources/OmniVLA/inference"
HAS_OMNIVLA = (OMNIVLA_SOURCES / "model_omnivla_edge.py").is_file()


class FakeNoMaD(torch.nn.Module):
    def forward(self, operation, **kwargs):
        if operation == "vision_encoder":
            assert not kwargs["input_goal_mask"].any()
            return torch.zeros(kwargs["obs_img"].shape[0], 256)
        if operation == "dist_pred_net":
            return torch.ones(kwargs["obsgoal_cond"].shape[0], 1)
        return torch.zeros_like(kwargs["sample"])


class WaypointTests(unittest.TestCase):
    def test_decode_bounds_and_cumulative_sum(self):
        sample = torch.tensor([[[-1., -1.], [1., 1.], [0., 0.]]])
        torch.testing.assert_close(decode_actions(sample),
                                   torch.tensor([[[-2.5, -4.], [2.5, 0.], [3.75, 0.]]]))

    def test_nomad_goal_conditioned_reproducible(self):
        model = NoMaDAdapter(FakeNoMaD(), {"len_traj_pred": 8, "num_diffusion_iters": 10})
        obs, goal = torch.zeros(1, 12, 96, 96), torch.zeros(1, 3, 96, 96)
        dist, action = model(obs, goal)
        self.assertEqual(action.shape, (1, 8, 2))
        torch.testing.assert_close(action, model(obs, goal)[1])
        self.assertTrue(torch.isfinite(action).all())

    def test_model_config(self):
        with (Path(__file__).resolve().parents[1] / "src/config/models.yaml").open() as f:
            conf = yaml.safe_load(f)
        self.assertEqual(conf["gnm_large"]["image_size"], [85, 64])
        self.assertEqual(conf["vint"]["len_traj_pred"], 5)
        self.assertEqual(conf["nomad"]["len_traj_pred"], 8)
        self.assertEqual(conf["nomad"]["image_size"], [96, 96])

    def test_omnivla_edge_config(self):
        with (ROOT / "src/config/models.yaml").open() as f:
            conf = yaml.safe_load(f)["omnivla_edge"]
        self.assertEqual(conf["image_size"], [96, 96])
        self.assertEqual((conf["context_size"], conf["len_traj_pred"]), (5, 8))
        self.assertEqual(conf["metric_scale"], 0.1)  # numeric: fixed metres per unit
        self.assertTrue(conf["learn_angle"])


WEIGHTS = ROOT / "src/model_weights/waypoint/omnivla_edge.pth"


class OmniVLAEdgePoseInputTests(unittest.TestCase):
    """Goal-pose conversion only; the network is not needed."""

    def setUp(self):
        from placenav.waypoint_backends import OmniVLAEdgeAdapter
        with (ROOT / "src/config/models.yaml").open() as f:
            conf = yaml.safe_load(f)["omnivla_edge"]
        self.adapter = OmniVLAEdgeAdapter(None, conf)

    def test_upstream_units_heading_and_batch(self):
        pose = self.adapter.pose_input((2.0, -0.5, math.pi / 2), 3, "cpu", torch.float32)
        self.assertEqual(pose.shape, (3, 4))
        torch.testing.assert_close(pose[0], torch.tensor([20.0, -5.0, 0.0, 1.0]), atol=1e-6, rtol=0)
        torch.testing.assert_close(pose[2], pose[0])

    def test_distance_clipped_like_upstream(self):
        far = self.adapter.pose_input((60.0, 80.0, 0.0), 1, "cpu", torch.float32)
        torch.testing.assert_close(far[0, :2], torch.tensor([180.0, 240.0]))  # 30 m along (0.6, 0.8)

    def test_non_finite_pose_rejected(self):
        for pose in ((float("nan"), 0.0, 0.0), (1.0, float("inf"), 0.0), (1.0, 0.0, float("nan"))):
            with self.subTest(pose=pose), self.assertRaises(ValueError):
                self.adapter.pose_input(pose, 1, "cpu", torch.float32)


# Real weights are required: a randomly initialised network ends in ReLU and
# outputs a constant, which would make every sensitivity check below vacuous.
@unittest.skipUnless(HAS_OMNIVLA and WEIGHTS.is_file() and torch.cuda.is_available(),
                     "needs CUDA and src/prepare_waypoint.py --models omnivla_edge")
class OmniVLAEdgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with (ROOT / "src/config/models.yaml").open() as f:
            conf = yaml.safe_load(f)["omnivla_edge"]
        cls.adapter = load_waypoint_backend(WEIGHTS, conf, "cuda")
        torch.manual_seed(0)
        cls.obs = torch.randn(2, 18, 96, 96).cuda()
        cls.goal = torch.randn(2, 3, 96, 96).cuda()

    def raw(self, modality, pose, text):
        ids = torch.full((2,), modality, dtype=torch.long, device="cuda")
        with torch.no_grad():
            action, dist, _ = self.adapter.model(
                self.obs, pose, torch.zeros(2, 9, 96, 96).cuda(), self.goal, ids, text,
                torch.zeros(2, 3, 224, 224).cuda())
        return action

    def test_shape_and_unit_heading(self):
        with torch.no_grad():
            dist, action = self.adapter(self.obs, self.goal)
        self.assertEqual(action.shape, (2, 8, 4))
        self.assertEqual(dist.shape, (2, 1))
        self.assertTrue(torch.isfinite(action).all() and torch.isfinite(dist).all())
        torch.testing.assert_close(action[..., 2:].norm(dim=-1), torch.ones(2, 8, device="cuda"))

    def test_unused_modalities_cannot_influence_goal_image_mode(self):
        """Zeros for pose/map/language are exact stand-ins: the network masks them."""
        with torch.no_grad():
            _, action = self.adapter(self.obs, self.goal)
        noisy = self.raw(6, torch.randn(2, 4).cuda(), torch.randn(2, 512).cuda())
        torch.testing.assert_close(action, noisy)
        # Negative control: a mode that does read the goal pose must react to it.
        zero = self.raw(4, torch.zeros(2, 4).cuda(), torch.zeros(2, 512).cuda())
        moved = self.raw(4, torch.randn(2, 4).cuda() * 20, torch.zeros(2, 512).cuda())
        self.assertFalse(torch.allclose(zero, moved), "control failed: pose mode ignores the pose")

    def test_goal_pose_runs_pose_and_image_mode(self):
        """adapter(..., goal_pose) == upstream modality 5 with the converted pose."""
        goal_pose = (2.5, 0.8, 0.3)
        with torch.no_grad():
            _, action = self.adapter(self.obs, self.goal, goal_pose=goal_pose)
        expected = torch.tensor([[25.0, 8.0, math.cos(0.3), math.sin(0.3)]] * 2, device="cuda")
        torch.testing.assert_close(action, self.raw(5, expected, torch.zeros(2, 512).cuda()))
        with torch.no_grad():
            _, image_only = self.adapter(self.obs, self.goal)
            _, moved = self.adapter(self.obs, self.goal, goal_pose=(2.5, -2.0, -0.6))
        self.assertFalse(torch.allclose(action, image_only), "pose mode must differ from image only")
        self.assertFalse(torch.allclose(action, moved), "pose mode must react to the pose")


if __name__ == "__main__":
    unittest.main()
