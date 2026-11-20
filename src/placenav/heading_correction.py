"""Heading measurements for LocalPoseTracker: how a correction is *estimated*.

Applying a measurement is LocalPoseTracker.apply_heading(); this module only measures
dpsi = (live robot heading) - (heading recorded at a topomap node), CCW positive, from
the live camera image and the node image. Estimators are interchangeable:

    class MyEstimator(HeadingEstimator):
        name = "my_method"
        def estimate(self, image_rgb, node_idx):
            ...  # return HeadingEstimate(dpsi, std, support) or None
    register_heading_estimator(MyEstimator)

and select it with navigate.py --heading-correction my_method. AsyncHeadingCorrector
runs any estimator off the navigation thread.
"""
from collections import OrderedDict
from dataclasses import dataclass
import math
from pathlib import Path
import queue
import threading
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class HeadingEstimate:
    """Result of one estimator call."""
    dpsi: float     # rad; live heading - node heading, CCW positive
    std: float      # rad; assumed 1-sigma of dpsi
    support: int    # estimator-specific evidence (e.g. agreeing matches)


@dataclass(frozen=True)
class HeadingRequest:
    """A live image to compare with a node, and the tracker state when it was taken."""
    node_idx: int
    stamp_ns: int
    epoch: object
    image: np.ndarray            # live RGB image (H, W, 3), uint8
    tracker_pose: tuple          # (x, y, yaw) of the tracker at stamp_ns, map frame


@dataclass(frozen=True)
class HeadingMeasurement:
    """An estimate tied to its request; what LocalPoseTracker.apply_heading consumes."""
    request: HeadingRequest
    estimate: HeadingEstimate
    method: str


class HeadingEstimator:
    """Base class. Subclasses set `name` and implement estimate()."""
    name = ""

    def __init__(self, topomap_dir, image_names, **options):
        self.topomap_dir = Path(topomap_dir)
        self.image_names = list(image_names)

    def node_image_path(self, node_idx):
        return self.topomap_dir / self.image_names[node_idx]

    def estimate(self, image_rgb, node_idx) -> Optional[HeadingEstimate]:
        raise NotImplementedError


_ESTIMATORS = {}


def register_heading_estimator(cls):
    if not cls.name:
        raise ValueError("Heading estimators need a name")
    _ESTIMATORS[cls.name] = cls
    return cls


def heading_estimator_names():
    return sorted(_ESTIMATORS)


def make_heading_estimator(name, topomap_dir, image_names, **options):
    if name not in _ESTIMATORS:
        raise ValueError(f"Unknown heading estimator {name!r}; choose from {heading_estimator_names()}")
    return _ESTIMATORS[name](topomap_dir, image_names, **options)


@register_heading_estimator
class SiftShiftHeadingEstimator(HeadingEstimator):
    """Dominant horizontal shift of SIFT matches between the live and the node image.

    Under an equidistant (angle-linear) lens model a pure rotation moves the scene by
    f * angle pixels horizontally; distant features dominate the mode of the shift
    histogram, so moderate translation (parallax of near features) is rejected.
    A simple stand-in: it assumes the principal point at the image centre and only
    a horizontal field of view (camera_hfov_deg) instead of a calibration.
    """
    name = "sift_shift"

    def __init__(self, topomap_dir, image_names, camera_hfov_deg, width=1440, nfeatures=8000,
                 min_support=10, std_deg=3.0, cache_size=32, **options):
        super().__init__(topomap_dir, image_names, **options)
        import cv2
        if not 0 < camera_hfov_deg < 360:
            raise ValueError("camera_hfov_deg must be in (0, 360)")
        self.cv2 = cv2
        self.width = int(width)
        self.f_px = self.width / math.radians(camera_hfov_deg)  # pixels per radian at `width`
        self.min_support = int(min_support)
        self.std = math.radians(std_deg)
        self.sift = cv2.SIFT_create(nfeatures=int(nfeatures))
        self.matcher = cv2.BFMatcher(cv2.NORM_L2)
        self.cache_size = int(cache_size)
        self._node_cache = OrderedDict()

    def _gray(self, image, bgr=False):
        cv2 = self.cv2
        g = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY if bgr else cv2.COLOR_RGB2GRAY)
        h = int(round(g.shape[0] * self.width / g.shape[1]))
        return cv2.resize(g, (self.width, h), interpolation=cv2.INTER_AREA)

    def _node_features(self, node_idx):
        if node_idx in self._node_cache:
            self._node_cache.move_to_end(node_idx)
            return self._node_cache[node_idx]
        bgr = self.cv2.imread(str(self.node_image_path(node_idx)), self.cv2.IMREAD_COLOR)
        if bgr is None:
            raise FileNotFoundError(self.node_image_path(node_idx))
        features = self.sift.detectAndCompute(self._gray(bgr, bgr=True), None)
        self._node_cache[node_idx] = features
        while len(self._node_cache) > self.cache_size:
            self._node_cache.popitem(last=False)
        return features

    def shift_px(self, node_features, live_features):
        """Dominant x_live - x_node [px at `width`] and its support, or (None, support)."""
        (kpa, da), (kpb, db) = node_features, live_features
        if da is None or db is None or len(kpa) < 20 or len(kpb) < 20:
            return None, 0
        pairs = self.matcher.knnMatch(da, db, k=2)
        good = [m for m, n in (p for p in pairs if len(p) == 2) if m.distance < 0.75 * n.distance]
        if len(good) < self.min_support:
            return None, len(good)
        dx = np.array([kpb[m.trainIdx].pt[0] - kpa[m.queryIdx].pt[0] for m in good])
        dy = np.array([kpb[m.trainIdx].pt[1] - kpa[m.queryIdx].pt[1] for m in good])
        dx = dx[np.abs(dy - np.median(dy)) < 40]  # same row band rejects most false matches
        if len(dx) < self.min_support:
            return None, len(dx)
        hist, edges = np.histogram(dx, bins=np.arange(-self.width, self.width + 8, 8))
        k = int(np.argmax(np.convolve(hist, np.ones(3), mode="same")))
        peak = 0.5 * (edges[k] + edges[k + 1])
        near = dx[np.abs(dx - peak) < 24]
        if len(near) < self.min_support:
            return None, len(near)
        return float(np.median(near)), int(len(near))

    def estimate(self, image_rgb, node_idx):
        live = self.sift.detectAndCompute(self._gray(image_rgb), None)
        px, support = self.shift_px(self._node_features(node_idx), live)
        if px is None:
            return None
        # The scene moves right (x grows) when the camera turns left (CCW): dpsi = +px / f.
        return HeadingEstimate(dpsi=px / self.f_px, std=self.std, support=support)


class AsyncHeadingCorrector:
    """Run an estimator on a worker thread; poll() returns finished measurements.

    Requests arrive at subgoal switches, a few seconds apart. If the worker is still
    busy, the oldest waiting request is dropped: a newer image of a newer node is
    more useful than a late one.
    """

    def __init__(self, estimator, max_pending=2, on_error=None):
        self.estimator = estimator
        self.on_error = on_error
        self._requests = queue.Queue(maxsize=max_pending)
        self._done = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="heading-correction", daemon=True)
        self._thread.start()

    def submit(self, request):
        while True:
            try:
                self._requests.put_nowait(request)
                return
            except queue.Full:
                try:
                    self._requests.get_nowait()
                except queue.Empty:
                    pass

    def poll(self):
        out = []
        while True:
            try:
                out.append(self._done.get_nowait())
            except queue.Empty:
                return out

    def close(self, timeout=1.0):
        self._stop.set()
        self._thread.join(timeout)

    def _run(self):
        while not self._stop.is_set():
            try:
                request = self._requests.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                estimate = self.estimator.estimate(request.image, request.node_idx)
            except Exception as error:  # keep navigating; report and skip this request
                if self.on_error is not None:
                    self.on_error(request, error)
                continue
            if estimate is not None:
                self._done.put(HeadingMeasurement(request, estimate, self.estimator.name))
