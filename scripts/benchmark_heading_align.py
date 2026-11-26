#!/usr/bin/env python3
"""Processing time (and accuracy) of the --heading-align measurement on this machine.

Runs the same estimator as navigate.py (placenav/rotation_estimation.py: SuperPoint+LightGlue,
rotation-only RANSAC) on pairs of topomap images: node k stands in for the live frame and
node k+step for the subgoal, so no bag is needed. The taught yaw difference of the pair
(node_odometry.json) is the reference; it carries the map run's odometry error, so read the
error columns as a sanity check and the time columns as the result.

Per configuration (SuperPoint resize : max keypoints) it reports the time of one live
measurement (live feature extraction + matching + RANSAC; node features are cached in
navigate.py and timed separately), the success rate and |dpsi - taught yaw difference|.

Inside the PlaceNav container on the robot PC (parrot), e.g.:
  python3 scripts/benchmark_heading_align.py \\
      --topomap-dir src/topomaps/images/whill_odometry_tsukuba_3.5_0.2618 --hfov 140
Optional: --images DIR times live frames from DIR (e.g. jpgs exported from a bag) against the
nodes instead (no reference angle).
"""
import argparse
import json
import math
from pathlib import Path
import statistics
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from placenav import rotation_estimation  # noqa: E402
from placenav.rotation_estimation import LightGlueRotationEstimator  # noqa: E402


def wrap_deg(a):
    return (a + 180.) % 360. - 180.


def percentile(values, q):
    return float(np.percentile(values, q)) if values else math.nan


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topomap-dir", type=Path, required=True)
    ap.add_argument("--hfov", type=float, default=140., help="camera_hfov_deg of the robot config")
    ap.add_argument("--configs", nargs="+", default=["640:512", "800:1024", "1024:1024", "1280:2048"],
                    help="RESIZE:MAX_KEYPOINTS")
    ap.add_argument("--pairs", type=int, default=60, help="node pairs per step (spread over the route)")
    ap.add_argument("--steps", type=int, nargs="+", default=[1, 2], help="node index steps of a pair")
    ap.add_argument("--images", type=Path, help="time live frames from this directory instead")
    ap.add_argument("--device", default=None)
    ap.add_argument("--weights-dir", type=Path, default=None)
    ap.add_argument("--json", type=Path, help="also write the summary here")
    args = ap.parse_args()

    meta = json.loads((args.topomap_dir / "node_odometry.json").read_text())
    nodes = meta["nodes"]
    names = [n["image"] for n in nodes]
    yaw = np.degrees(np.unwrap([n["pose"]["yaw"] for n in nodes]))
    rng = np.random.default_rng(0)
    pairs = []
    for step in args.steps:
        ks = np.arange(0, len(nodes) - step)
        for k in sorted(rng.choice(ks, size=min(args.pairs, len(ks)), replace=False)):
            pairs.append((int(k), int(k + step), float(yaw[k] - yaw[k + step])))  # dpsi = live - node
    import cv2
    import torch
    live_images = None
    if args.images:
        live_images = sorted(p for p in args.images.iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
        if not live_images:
            raise SystemExit(f"no images in {args.images}")

    def sync():
        if torch.cuda.is_available():
            torch.cuda.synchronize()

    device = args.device or ("cuda" if torch.cuda.is_available() else "cpu")
    gpu = torch.cuda.get_device_name(0) if device.startswith("cuda") else "cpu"
    print(f"device {device} ({gpu}), torch {torch.__version__}, {len(nodes)} nodes, {len(pairs)} pairs")
    summary = {"device": gpu, "torch": torch.__version__, "configs": {}}
    header = (f"{'resize:kp':>10s} | live total ms med/p90 | extract | match | ransac | node extract | "
              f"ok   | |err| med/p90 (all) | (|taught|>=15)")
    print(header)
    for spec in args.configs:
        resize, max_kp = (int(v) for v in spec.split(":"))
        est = LightGlueRotationEstimator(args.topomap_dir, names, camera_hfov_deg=args.hfov, resize=resize,
                                         max_keypoints=max_kp, device=device, weights_dir=args.weights_dir,
                                         cache_size=4 * len(pairs) + 8)
        original_ransac = rotation_estimation.rotation_ransac
        ransac_ms = []

        def timed_ransac(*a, **k):
            start = time.perf_counter()
            out = original_ransac(*a, **k)
            ransac_ms.append(1000 * (time.perf_counter() - start))
            return out
        rotation_estimation.rotation_ransac = timed_ransac
        try:
            # warm-up (CUDA kernels, cudnn autotune)
            for k, j, _ in pairs[:3]:
                est.estimate_from_features(est.node_features(j), est.node_features(k))
            ransac_ms.clear()
            node_ms, extract_ms, match_ms, total_ms, errors, errors_turn, ok = [], [], [], [], [], [], 0
            est._node_cache.clear()
            for idx, (k, j, taught) in enumerate(pairs):
                sync(); t0 = time.perf_counter()
                node = est.node_features(j)
                sync(); node_ms.append(1000 * (time.perf_counter() - t0))
                if live_images is not None:
                    bgr = cv2.imread(str(live_images[idx % len(live_images)]), cv2.IMREAD_COLOR)
                else:
                    bgr = cv2.imread(str(args.topomap_dir / names[k]), cv2.IMREAD_COLOR)
                rgb = np.ascontiguousarray(bgr[:, :, ::-1])
                sync(); t1 = time.perf_counter()
                live = est.live_features(rgb)
                sync(); t2 = time.perf_counter()
                n_ransac = len(ransac_ms)
                result = est.estimate_from_features(node, live)
                sync(); t3 = time.perf_counter()
                r_ms = ransac_ms[-1] if len(ransac_ms) > n_ransac else 0.
                extract_ms.append(1000 * (t2 - t1)); total_ms.append(1000 * (t3 - t1))
                match_ms.append(1000 * (t3 - t2) - r_ms)
                if result is not None:
                    ok += 1
                    if live_images is None:
                        err = abs(wrap_deg(math.degrees(result.dpsi) - taught))
                        errors.append(err)
                        if abs(taught) >= 15:
                            errors_turn.append(err)
        finally:
            rotation_estimation.rotation_ransac = original_ransac
        row = {
            "live_total_ms": [statistics.median(total_ms), percentile(total_ms, 90)],
            "extract_ms": statistics.median(extract_ms), "match_ms": statistics.median(match_ms),
            "ransac_ms": statistics.median(ransac_ms) if ransac_ms else math.nan,
            "node_extract_ms": statistics.median(node_ms), "success": ok / len(pairs),
            "err_deg": [percentile(errors, 50), percentile(errors, 90)],
            "err_turn_deg": [percentile(errors_turn, 50), percentile(errors_turn, 90), len(errors_turn)],
        }
        summary["configs"][spec] = row
        print(f"{spec:>10s} | {row['live_total_ms'][0]:7.1f} / {row['live_total_ms'][1]:7.1f}   | "
              f"{row['extract_ms']:7.1f} | {row['match_ms']:5.1f} | {row['ransac_ms']:6.1f} | "
              f"{row['node_extract_ms']:12.1f} | {100 * row['success']:3.0f}% | "
              f"{row['err_deg'][0]:5.1f} / {row['err_deg'][1]:5.1f}       | "
              f"{row['err_turn_deg'][0]:5.1f} / {row['err_turn_deg'][1]:5.1f} (n={row['err_turn_deg'][2]})", flush=True)
        del est
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
    if args.json:
        args.json.write_text(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
