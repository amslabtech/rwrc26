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

With --bag, the live frames come from a recorded run instead: each sampled camera frame is
paired with the subgoal PlaceNav had at that image (/toponav/goal_evaluation: query_timestamp,
subgoal_idx), i.e. exactly the pair --heading-align measures on arrival. If the bag has no
/toponav topics (e.g. a drive recorded without PlaceNav), give a bag that has them for the same
images with --toponav-bag (e.g. the nav_run.bag of an offline replay). There is no reference
angle then; the output adds the |dpsi| distribution and how many frames would start a rotation.

Runs anywhere the PlaceNav container runs (GPU recommended, CPU works) after
`pip install -r requirements-heading-align.txt`. On the robot PC (parrot), e.g.:
  python3 scripts/benchmark_heading_align.py \\
      --topomap-dir src/topomaps/images/whill_odometry_tsukuba_3.5_0.2618 --hfov 140
  python3 scripts/benchmark_heading_align.py --topomap-dir src/topomaps/images/<route> \\
      --bag <navigation bag of that route>        # needs ROS (rosbag) in the environment
Optional: --images DIR times live frames from DIR against the nodes (timing only).
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


def load_rgb(source, cv2):
    """(RGB image, JPEG decode ms or None). source: ("file", path) or ("jpeg", bytes) or ("raw", array)."""
    kind, value = source
    if kind == "file":
        bgr = cv2.imread(str(value), cv2.IMREAD_COLOR)
        return np.ascontiguousarray(bgr[:, :, ::-1]), None
    if kind == "raw":
        return value, None
    start = time.perf_counter()
    bgr = cv2.imdecode(np.frombuffer(value, np.uint8), cv2.IMREAD_COLOR)
    return np.ascontiguousarray(bgr[:, :, ::-1]), 1000 * (time.perf_counter() - start)


def bag_pairs(args, node_count):
    """Camera frames of --bag paired with the subgoal PlaceNav had for that image."""
    import rosbag
    toponav = args.toponav_bag or args.bag
    subgoal = {}
    with rosbag.Bag(str(toponav)) as bag:
        for _, msg, _ in bag.read_messages(topics=["/toponav/goal_evaluation"]):
            subgoal[msg.query_timestamp.to_nsec()] = int(msg.subgoal_idx)
    if not subgoal:
        raise SystemExit(f"{toponav} has no /toponav/goal_evaluation; pass --toponav-bag with PlaceNav's "
                         "outputs for these images (e.g. an offline replay's nav_run.bag)")
    stamps = np.array(sorted(subgoal))

    def node_of(ns):
        i = int(np.searchsorted(stamps, ns))
        near = [int(stamps[k]) for k in (i - 1, i) if 0 <= k < len(stamps)]
        best = min(near, key=lambda s: abs(s - ns), default=None)
        if best is None or abs(best - ns) > 50_000_000:  # an image PlaceNav did not process
            return None
        node = subgoal[best]
        return node if node < node_count else None

    # Pass 1: which frames have a subgoal (keeps no image data); pass 2: read the sampled ones.
    candidates = []
    with rosbag.Bag(str(args.bag)) as bag:
        for _, msg, _ in bag.read_messages(topics=[args.image_topic]):
            ns = msg.header.stamp.to_nsec()
            if node_of(ns) is not None:
                candidates.append(ns)
    if not candidates:
        raise SystemExit(f"no {args.image_topic} frames of {args.bag} match the PlaceNav image stamps")
    pick = {candidates[k] for k in np.linspace(0, len(candidates) - 1,
                                               min(args.max_frames, len(candidates))).round().astype(int)}
    frames = []
    with rosbag.Bag(str(args.bag)) as bag:
        for _, msg, _ in bag.read_messages(topics=[args.image_topic]):
            ns = msg.header.stamp.to_nsec()
            if ns not in pick:
                continue
            if hasattr(msg, "format"):  # sensor_msgs/CompressedImage
                frames.append((("jpeg", bytes(msg.data)), node_of(ns), None))
            else:
                from cv_bridge import CvBridge
                frames.append((("raw", CvBridge().imgmsg_to_cv2(msg, desired_encoding="rgb8")), node_of(ns), None))
    print(f"--bag: {len(candidates)} frames with a subgoal, using {len(frames)}")
    return frames


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topomap-dir", type=Path, required=True)
    ap.add_argument("--hfov", type=float, default=140., help="camera_hfov_deg of the robot config")
    ap.add_argument("--configs", nargs="+", default=["640:512", "800:1024", "1024:1024", "1280:2048"],
                    help="RESIZE:MAX_KEYPOINTS")
    ap.add_argument("--pairs", type=int, default=60, help="node pairs per step (spread over the route)")
    ap.add_argument("--steps", type=int, nargs="+", default=[1, 2], help="node index steps of a pair")
    ap.add_argument("--images", type=Path, help="time live frames from this directory instead")
    ap.add_argument("--bag", type=Path, help="live frames from this run, paired with its subgoals")
    ap.add_argument("--toponav-bag", type=Path, help="bag with /toponav/goal_evaluation for --bag's images")
    ap.add_argument("--image-topic", default="/usb_cam/image_raw/compressed")
    ap.add_argument("--max-frames", type=int, default=150, help="--bag: frames sampled evenly over the run")
    ap.add_argument("--start-align-deg", type=float, default=25., help="--bag: report frames above this |dpsi|")
    ap.add_argument("--device", default=None)
    ap.add_argument("--weights-dir", type=Path, default=None)
    ap.add_argument("--json", type=Path, help="also write the summary here")
    args = ap.parse_args()

    meta = json.loads((args.topomap_dir / "node_odometry.json").read_text())
    nodes = meta["nodes"]
    names = [n["image"] for n in nodes]
    yaw = np.degrees(np.unwrap([n["pose"]["yaw"] for n in nodes]))
    rng = np.random.default_rng(0)
    import cv2
    import torch
    # pairs: (live image source, node index, reference dpsi in degrees or None)
    pairs = []
    if args.bag:
        pairs = bag_pairs(args, len(nodes))
    elif args.images:
        live_images = sorted(p for p in args.images.iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
        if not live_images:
            raise SystemExit(f"no images in {args.images}")
        for idx, k in enumerate(rng.choice(len(nodes), size=min(args.pairs, len(nodes)), replace=False)):
            pairs.append((("file", live_images[idx % len(live_images)]), int(k), None))
    else:
        for step in args.steps:
            ks = np.arange(0, len(nodes) - step)
            for k in sorted(rng.choice(ks, size=min(args.pairs, len(ks)), replace=False)):
                pairs.append((("file", args.topomap_dir / names[k]), int(k + step),
                              float(yaw[k] - yaw[k + step])))  # dpsi = live - node

    def sync():
        if torch.cuda.is_available():
            torch.cuda.synchronize()

    device = args.device or ("cuda" if torch.cuda.is_available() else "cpu")
    gpu = torch.cuda.get_device_name(0) if device.startswith("cuda") else "cpu"
    print(f"device {device} ({gpu}), torch {torch.__version__}, {len(nodes)} nodes, {len(pairs)} pairs")
    summary = {"device": gpu, "torch": torch.__version__, "configs": {}}
    with_ref = any(ref is not None for _, _, ref in pairs)
    header = (f"{'resize:kp':>10s} | live total ms med/p90 | extract | match | ransac | node extract | ok   | "
              + ("|err| med/p90 (all) | (|taught|>=15)" if with_ref else
                 f"|dpsi| med/p90/max | > {args.start_align_deg:.0f} deg (any frame, not only arrivals) | jpeg decode ms"))
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
            for source, j, _ in pairs[:3]:
                est.estimate_from_features(est.node_features(j), est.live_features(load_rgb(source, cv2)[0]))
            ransac_ms.clear()
            node_ms, extract_ms, match_ms, total_ms, errors, errors_turn, ok = [], [], [], [], [], [], 0
            decode_ms, dpsis = [], []
            est._node_cache.clear()
            for idx, (source, j, taught) in enumerate(pairs):
                cached = j in est._node_cache
                sync(); t0 = time.perf_counter()
                node = est.node_features(j)
                sync()
                if not cached:  # navigate.py extracts each node once and caches it
                    node_ms.append(1000 * (time.perf_counter() - t0))
                rgb, decode = load_rgb(source, cv2)
                if decode is not None:
                    decode_ms.append(decode)
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
                    dpsis.append(abs(math.degrees(result.dpsi)))
                    if taught is not None:
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
            "node_extract_ms": statistics.median(node_ms) if node_ms else math.nan, "success": ok / len(pairs),
            "err_deg": [percentile(errors, 50), percentile(errors, 90)],
            "err_turn_deg": [percentile(errors_turn, 50), percentile(errors_turn, 90), len(errors_turn)],
            "abs_dpsi_deg": [percentile(dpsis, 50), percentile(dpsis, 90), max(dpsis, default=math.nan)],
            "would_rotate": sum(d > args.start_align_deg for d in dpsis),
            "jpeg_decode_ms": statistics.median(decode_ms) if decode_ms else math.nan, "pairs": len(pairs),
        }
        summary["configs"][spec] = row
        print(f"{spec:>10s} | {row['live_total_ms'][0]:7.1f} / {row['live_total_ms'][1]:7.1f}   | "
              f"{row['extract_ms']:7.1f} | {row['match_ms']:5.1f} | {row['ransac_ms']:6.1f} | "
              f"{row['node_extract_ms']:12.1f} | {100 * row['success']:3.0f}% | " +
              (f"{row['err_deg'][0]:5.1f} / {row['err_deg'][1]:5.1f}       | "
               f"{row['err_turn_deg'][0]:5.1f} / {row['err_turn_deg'][1]:5.1f} (n={row['err_turn_deg'][2]})" if with_ref else
               f"{row['abs_dpsi_deg'][0]:5.1f} / {row['abs_dpsi_deg'][1]:5.1f} / {row['abs_dpsi_deg'][2]:5.1f} | "
               f"{row['would_rotate']:3d} of {len(pairs)} frames          | {row['jpeg_decode_ms']:5.1f}"), flush=True)
        del est
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
    if args.json:
        args.json.write_text(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
