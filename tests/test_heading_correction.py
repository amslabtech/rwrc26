"""Heading estimators and the asynchronous runner (synthetic images, no map data)."""
import math
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from placenav.heading_correction import (AsyncHeadingCorrector, HeadingEstimate, HeadingEstimator,
                                         HeadingRequest, heading_estimator_names, make_heading_estimator,
                                         register_heading_estimator)

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


def texture(seed, width=1600, height=930):
    """Multi-scale random texture with blobs: plenty of SIFT keypoints."""
    rng = np.random.default_rng(seed)
    image = np.zeros((height, width), np.float32)
    for scale in (3, 9, 27):
        noise = rng.normal(size=(height, width)).astype(np.float32)
        image += cv2.GaussianBlur(noise, (0, 0), scale) * scale
    for _ in range(300):
        cv2.circle(image, (int(rng.integers(width)), int(rng.integers(height))),
                   int(rng.integers(4, 30)), float(rng.normal() * 40), -1)
    image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    return np.dstack([image] * 3)


class RegistryTest(unittest.TestCase):
    def test_unknown_estimator_rejected_and_custom_one_selectable(self):
        with self.assertRaises(ValueError):
            make_heading_estimator('no_such_method', '.', [])

        @register_heading_estimator
        class Constant(HeadingEstimator):
            name = 'test_constant'

            def estimate(self, image_rgb, node_idx):
                return HeadingEstimate(dpsi=.1, std=.01, support=1)

        self.assertIn('test_constant', heading_estimator_names())
        estimator = make_heading_estimator('test_constant', '.', ['0.jpg'])
        self.assertEqual(estimator.estimate(None, 0).dpsi, .1)


@unittest.skipIf(cv2 is None, 'needs OpenCV')
class SiftShiftTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.dir = Path(self.temp.name)
        self.scene = texture(0)  # 1600 px wide; the node image is columns 100..1539
        self.node = np.ascontiguousarray(self.scene[:, 100:1540])
        cv2.imwrite(str(self.dir/'0.jpg'), self.node[:, :, ::-1])
        self.estimator = make_heading_estimator('sift_shift', self.dir, ['0.jpg'], camera_hfov_deg=140.0)

    def tearDown(self):
        self.temp.cleanup()

    def test_horizontal_shift_gives_heading_with_sign(self):
        f = 1440 / math.radians(140.0)
        for shift in (-60, -12, 0, 25, 80):
            with self.subTest(shift=shift):
                # The live camera turned left (CCW) by shift/f, so the scene moved right by
                # `shift` px: live[:, x] = node[:, x - shift].
                live = np.ascontiguousarray(self.scene[:, 100 - shift:1540 - shift])
                estimate = self.estimator.estimate(live, 0)
                self.assertIsNotNone(estimate)
                self.assertAlmostEqual(math.degrees(estimate.dpsi), math.degrees(shift / f), delta=0.3)
                self.assertGreater(estimate.support, 50)

    def test_unrelated_image_gives_no_estimate(self):
        self.assertIsNone(self.estimator.estimate(np.ascontiguousarray(texture(1)[:, :1440]), 0))

    def test_node_features_are_cached(self):
        self.estimator.estimate(self.node, 0)
        (self.dir/'0.jpg').unlink()  # a second call must not reread the node image
        self.assertIsNotNone(self.estimator.estimate(self.node, 0))


class AsyncCorrectorTest(unittest.TestCase):
    def request(self, node):
        return HeadingRequest(node_idx=node, stamp_ns=node, epoch=0, image=None, tracker_pose=(0., 0., 0.))

    def wait(self, corrector, count, timeout=2.0):
        out, deadline = [], time.time() + timeout
        while len(out) < count and time.time() < deadline:
            out += corrector.poll()
            time.sleep(.01)
        return out

    def test_results_keep_their_request_and_failures_are_reported(self):
        class Estimator(HeadingEstimator):
            name = 'test_async'

            def estimate(self, image_rgb, node_idx):
                if node_idx == 2:
                    raise RuntimeError('boom')
                return None if node_idx == 3 else HeadingEstimate(node_idx / 10, .01, 5)

        errors = []
        corrector = AsyncHeadingCorrector(Estimator('.', []), on_error=lambda r, e: errors.append(r.node_idx))
        try:
            for node in (1, 2, 3, 4):
                corrector.submit(self.request(node))
                time.sleep(.05)
            done = self.wait(corrector, 2)
        finally:
            corrector.close()
        self.assertEqual([(m.request.node_idx, m.estimate.dpsi, m.method) for m in done],
                         [(1, .1, 'test_async'), (4, .4, 'test_async')])
        self.assertEqual(errors, [2])

    def test_busy_worker_drops_the_oldest_waiting_request(self):
        release = threading.Event()

        class Slow(HeadingEstimator):
            name = 'test_slow'

            def estimate(self, image_rgb, node_idx):
                release.wait(2)
                return HeadingEstimate(0., .01, 1)

        corrector = AsyncHeadingCorrector(Slow('.', []), max_pending=1)
        try:
            corrector.submit(self.request(1))
            time.sleep(.2)                       # worker is now busy with node 1
            for node in (2, 3, 4):
                corrector.submit(self.request(node))
            release.set()
            done = self.wait(corrector, 2)
        finally:
            corrector.close()
        self.assertEqual([m.request.node_idx for m in done], [1, 4])


if __name__ == '__main__':
    unittest.main()
