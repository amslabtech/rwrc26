#!/usr/bin/env python3
"""Re-run image-goal OmniVLA on recorded C2 queries, without a ROS master."""
import argparse
import math
import os
from collections import deque
from functools import lru_cache
from pathlib import Path
from tempfile import TemporaryDirectory

import cv2
import numpy as np
import rosbag
from genpy import Time
import torch
import yaml

from placenav.waypoint_backends import load_waypoint_backend, waypoint_transform

ROOT = Path(__file__).resolve().parent
CAMERA = "/usb_cam/image_raw/compressed"
VIZ = "/toponav/viz_info"


def compose_frame(image, xy, recorded, elapsed, goal):
    """Keep a metric base_link view even when the path is below the camera FOV."""
    panel = np.full((440, 960, 3), 32, np.uint8)
    h, w = image.shape[:2]
    factor = min(640 / w, 385 / h)
    small = cv2.resize(image, (round(w * factor), round(h * factor)),
                       interpolation=cv2.INTER_AREA)
    height, width = small.shape[:2]
    left = (640 - width) // 2
    panel[40:40 + height, left:left + width] = small

    def label(value, x, y):
        cv2.putText(panel, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (235, 235, 235), 1, cv2.LINE_AA)

    label(f"C2    bag {elapsed:.2f} s    goal {goal}", 10, 28)
    label("base_link    forward up / left left", 650, 28)
    extent = max(1.0, np.abs(xy).max(), np.abs(recorded).max())
    scale = 132 / extent
    origin = np.array([800, 265])
    cv2.line(panel, (650, 265), (950, 265), (100, 100, 100), 1)
    cv2.line(panel, (800, 110), (800, 405), (100, 100, 100), 1)
    for path, color, thickness in [(recorded, (240, 160, 60), 6), (xy, (80, 230, 110), 2)]:
        points = np.vstack([[0, 0], path])
        pixels = np.rint(origin - scale * points[:, ::-1]).astype(np.int32)
        cv2.polylines(panel, [pixels], False, color, thickness, cv2.LINE_AA)
        for point in pixels[1:]:
            cv2.circle(panel, tuple(point), thickness // 2 + 1, color, -1)
    label("green: replay   blue: recorded", 650, 58)
    label(f"end: x {xy[-1, 0]:+.2f}  y {xy[-1, 1]:+.2f} m", 650, 82)
    cv2.line(panel, (660, 413), (660 + round(0.5 * scale), 413), (220, 220, 220), 2)
    label("0.5 m", 660, 435)
    return panel


def render(args, output):
    config = yaml.safe_load((ROOT / "config/models.yaml").read_text())["omnivla_edge"]
    transform = waypoint_transform(config["image_size"])
    model = load_waypoint_backend(args.weights_dir / config["checkpoint_path"], config, "cuda")
    files = sorted((f for f in args.topomap.iterdir() if f.suffix in (".jpg", ".jpeg", ".png")),
                   key=lambda f: int(f.stem))

    @lru_cache(maxsize=8)
    def goal_image(index):
        if not 0 <= index < len(files):
            raise ValueError(f"Goal {index} outside supplied map ({len(files)} images)")
        image = cv2.imread(str(files[index]))
        if image is None:
            raise ValueError(f"Unreadable goal image: {files[index]}")
        return transform(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)).unsqueeze(0).cuda()

    history = deque(maxlen=config["context_size"] + 1)
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), 5, (960, 440))
    if not writer.isOpened():
        raise RuntimeError(f"Cannot open video output {output}")
    errors = []
    endpoints = []
    try:
        with rosbag.Bag(str(args.bag)) as bag, torch.inference_mode():
            origin = bag.get_start_time()
            first = origin + args.start
            last = first + args.duration if args.duration else bag.get_end_time()
            queries = {}
            for _, msg, _ in bag.read_messages([VIZ], Time.from_sec(first), Time.from_sec(last)):
                if msg.waypoints:
                    queries[msg.query_timestamp.to_nsec()] = (
                        msg.subgoal_idx, np.array([(p.x, p.y) for p in msg.waypoints]))
            if not queries:
                raise ValueError("No recorded non-terminal PlaceNav queries in this interval")
            # Include history before the selected interval. Match by camera acquisition
            # timestamp; a late/missing recording must not be paired to another image.
            begin = max(0.0, min(first, min(queries) / 1e9) - 3)
            end = max(last, max(queries) / 1e9) + 1
            previous_stamp = None
            seen = set()
            for _, msg, _ in bag.read_messages([CAMERA], Time.from_sec(begin), Time.from_sec(end)):
                stamp = msg.header.stamp.to_nsec()
                if previous_stamp is not None and stamp < previous_stamp:
                    history.clear()
                previous_stamp = stamp
                image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    history.clear()
                    continue
                history.append(transform(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)))
                if stamp not in queries or stamp in seen or len(history) < history.maxlen:
                    continue
                goal, recorded = queries[stamp]
                context = torch.cat(list(history), dim=0).unsqueeze(0).cuda()
                _, action = model(context, goal_image(goal))
                xy = action[0, :, :2].cpu().numpy() * config["metric_scale"]
                if xy.shape != recorded.shape or not np.isfinite(xy).all() or not np.isfinite(recorded).all():
                    raise ValueError(f"Invalid prediction or reference at {stamp}")
                writer.write(compose_frame(image, xy, recorded, stamp / 1e9 - origin, goal))
                errors.append(np.linalg.norm(xy - recorded, axis=1))
                endpoints.append(xy[-1])
                seen.add(stamp)
    finally:
        writer.release()
    if not errors:
        raise ValueError("No matching C2 frames with sufficient history")
    end_xy = np.array(endpoints)
    print(f"Matched {len(errors)}/{len(queries)} queries; median point difference from bag "
          f"{np.median(errors):.3f} m; endpoint y (left positive): "
          f"{end_xy[:, 1].min():+.3f} .. {end_xy[:, 1].max():+.3f} m")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--topomap", required=True, type=Path, help="Same image directory as the recorded run")
    parser.add_argument("--output", required=True, type=Path, help="New .mp4 file")
    parser.add_argument("--start", type=float, default=0, help="Seconds from bag start")
    parser.add_argument("--duration", type=float, help="Seconds of recorded queries")
    parser.add_argument("--weights-dir", type=Path, default=ROOT / "model_weights")
    args = parser.parse_args()
    if args.output.suffix.lower() != ".mp4" or args.output.exists():
        parser.error("--output must name a new .mp4 file")
    if not math.isfinite(args.start) or args.start < 0:
        parser.error("start must be a finite nonnegative number")
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error("duration must be a finite positive number")
    torch.set_num_threads(2)
    with TemporaryDirectory(dir=args.output.parent) as folder:
        temp = Path(folder) / "preview.mp4"
        render(args, temp)
        os.link(temp, args.output)  # Publish the complete file without replacing another run.


if __name__ == "__main__":
    main()
