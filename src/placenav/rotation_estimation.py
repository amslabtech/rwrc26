"""Heading difference between a live image and a topomap node image from feature matches.

SuperPoint+LightGlue matches are turned into unit bearing vectors with an
equidistant fisheye model (theta = r / f, principal point at the image centre;
f from the robot config's horizontal FOV). The two views are assumed to differ
only by a rotation: near the node position the baseline (<~1 m) is small
compared with the depth of most matched points, and the essential-matrix
(5/8-point) solution is ill-conditioned there. RANSAC on 2-point Kabsch fits
rejects near points with parallax and wrong matches. The yaw is the rotation
about the robot's vertical axis, positive when the live view is turned left
(CCW) of the node view, as `HeadingEstimate.dpsi`.

Only numpy/cv2 at import; torch, kornia and LightGlue load when an estimator is built.
"""
from collections import OrderedDict
import math
from pathlib import Path

import numpy as np

from placenav.heading_correction import HeadingEstimate, HeadingEstimator, register_heading_estimator

# Robot vertical in the optical frame (x right, y down, z forward) of a level camera.
LEVEL_UP = np.array([0., -1., 0.])


def equidistant_bearings(px, f_px, cx, cy):
    """Pixels (N, 2) -> unit bearing vectors (N, 3) in the optical frame."""
    d = np.asarray(px, dtype=float) - np.array([cx, cy])
    r = np.maximum(np.linalg.norm(d, axis=1), 1e-9)
    theta = r / f_px
    s = np.sin(theta) / r
    return np.column_stack([d[:, 0] * s, d[:, 1] * s, np.cos(theta)])


def kabsch(a, b):
    """Rotation(s) R with b ~ R a. a, b: (..., N, 3)."""
    h = np.swapaxes(b, -1, -2) @ a
    u, _, vt = np.linalg.svd(h)
    det = np.sign(np.linalg.det(u @ vt))
    d = np.ones(u.shape[:-2] + (3,))
    d[..., 2] = det
    return (u * d[..., None, :]) @ vt


def rotation_ransac(a, b, threshold_deg=1.0, iterations=500, rng=None):
    """Rotation R with b ~ R a from bearing pairs, robust to outliers. Returns (R, inlier mask) or None."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    n = len(a)
    if n < 3:
        return None
    rng = rng if rng is not None else np.random.default_rng(0)
    i = rng.integers(0, n, size=(iterations, 2))
    i = i[i[:, 0] != i[:, 1]]
    if len(i) == 0:
        return None
    rotations = kabsch(a[i], b[i])                       # (K, 3, 3)
    cos_threshold = math.cos(math.radians(threshold_deg))
    predicted = np.einsum('kij,nj->kni', rotations, a)   # (K, N, 3)
    support = (np.einsum('kni,ni->kn', predicted, b) > cos_threshold).sum(1)
    rotation = rotations[int(np.argmax(support))]
    for _ in range(2):  # refit on the consensus set
        inliers = (a @ rotation.T * b).sum(1) > cos_threshold
        if inliers.sum() < 3:
            return None
        rotation = kabsch(a[inliers], b[inliers])
    inliers = (a @ rotation.T * b).sum(1) > cos_threshold
    return rotation, inliers


def yaw_about(rotation, up=LEVEL_UP):
    """Yaw (rad, CCW +) of the live camera relative to the node camera, b = R a (node -> live bearings)."""
    relative = rotation.T  # live camera orientation in the node camera frame
    angle = math.acos(max(-1., min(1., (np.trace(relative) - 1) / 2)))
    if angle < 1e-9:
        return 0.
    axis = np.array([relative[2, 1] - relative[1, 2], relative[0, 2] - relative[2, 0],
                     relative[1, 0] - relative[0, 1]]) / (2 * math.sin(angle))
    return float(angle * axis @ (np.asarray(up) / np.linalg.norm(up)))


class LightGlueRotationEstimator(HeadingEstimator):
    """dpsi from SuperPoint+LightGlue matches and a rotation-only model (see module docstring)."""
    name = "lg_rotation"

    def __init__(self, topomap_dir, image_names, camera_hfov_deg, resize=1024, max_keypoints=1024,
                 min_inliers=20, threshold_deg=1.0, iterations=500, std_deg=3.0, weights_dir=None,
                 device=None, cache_size=48, **options):
        super().__init__(topomap_dir, image_names, **options)
        import cv2
        import torch
        if not 0 < camera_hfov_deg < 360:
            raise ValueError("camera_hfov_deg must be in (0, 360)")
        from lightglue import LightGlue, SuperPoint
        self.cv2, self.torch = cv2, torch
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        # LightGlue loads its weights through torch.hub (<hub dir>/checkpoints/*.pth, downloaded
        # if missing). PlaceNav's hub dir is src/model_weights/torch_hub_checkpoints; another
        # weights_dir is used only while the models load, then the global hub dir is restored.
        previous = torch.hub.get_dir()
        try:
            if weights_dir is not None:
                torch.hub.set_dir(str(Path(weights_dir)))
            self.extractor = SuperPoint(max_num_keypoints=int(max_keypoints)).eval().to(self.device)
            self.matcher = LightGlue(features="superpoint").eval().to(self.device)
        finally:
            torch.hub.set_dir(previous)
        self.hfov = math.radians(camera_hfov_deg)
        self.resize = int(resize)
        self.min_inliers = int(min_inliers)
        self.threshold_deg = float(threshold_deg)
        self.iterations = int(iterations)
        self.std = math.radians(std_deg)
        self.rng = np.random.default_rng(0)
        self.cache_size = int(cache_size)
        self._node_cache = OrderedDict()
        self.last = {}  # diagnostics of the latest estimate

    def _features(self, rgb):
        image = self.torch.from_numpy(np.ascontiguousarray(rgb)).permute(2, 0, 1).float().div(255).to(self.device)
        with self.torch.no_grad():
            return self.extractor.extract(image, resize=self.resize)

    def node_features(self, node_idx):
        if node_idx in self._node_cache:
            self._node_cache.move_to_end(node_idx)
            return self._node_cache[node_idx]
        bgr = self.cv2.imread(str(self.node_image_path(node_idx)), self.cv2.IMREAD_COLOR)
        if bgr is None:
            raise FileNotFoundError(self.node_image_path(node_idx))
        entry = (self._features(bgr[:, :, ::-1]), bgr.shape[1], bgr.shape[0])
        self._node_cache[node_idx] = entry
        while len(self._node_cache) > self.cache_size:
            self._node_cache.popitem(last=False)
        return entry

    def live_features(self, image_rgb):
        return self._features(image_rgb), image_rgb.shape[1], image_rgb.shape[0]

    def estimate_from_features(self, node, live):
        (fn, wn, hn), (fl, wl, hl) = node, live
        with self.torch.no_grad():
            matches = self.matcher({"image0": fn, "image1": fl})["matches"][0].cpu().numpy()
        self.last = {"matches": int(len(matches)), "inliers": 0}
        if len(matches) < self.min_inliers:
            return None
        pn = fn["keypoints"][0].cpu().numpy()[matches[:, 0]]
        pl = fl["keypoints"][0].cpu().numpy()[matches[:, 1]]
        # Same camera; f scales with the stored resolution (node images may be resized).
        a = equidistant_bearings(pn, wn / self.hfov, wn / 2, hn / 2)
        b = equidistant_bearings(pl, wl / self.hfov, wl / 2, hl / 2)
        fit = rotation_ransac(a, b, self.threshold_deg, self.iterations, self.rng)
        if fit is None:
            return None
        rotation, inliers = fit
        self.last["inliers"] = int(inliers.sum())
        if inliers.sum() < self.min_inliers:
            return None
        return HeadingEstimate(dpsi=yaw_about(rotation), std=self.std, support=int(inliers.sum()))

    def estimate(self, image_rgb, node_idx):
        return self.estimate_from_features(self.node_features(node_idx), self.live_features(image_rgb))


register_heading_estimator(LightGlueRotationEstimator)
