"""Metric route prediction for the topological Bayes filter (no ROS imports)."""
from collections import deque
import json
import math
from pathlib import Path

import numpy as np
from scipy.special import ndtr

from placenav.topomap_odometry import interpolate_odometry, relative_pose


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


class OdometryHistory:
    """Bounded history; publication gaps do not reset a continuous odom frame.

    at() separately limits interpolation. Plausible endpoint deltas can still
    be integrated after a gap; frame/time/pose discontinuities start an epoch.
    """
    def __init__(self, max_gap=0.2, max_speed=float('inf')):
        if not math.isfinite(max_gap) or max_gap <= 0:
            raise ValueError("odom_max_gap must be finite and positive")
        self.max_gap_ns = int(max_gap * 1e9)
        if math.isnan(max_speed) or max_speed <= 0:
            raise ValueError("Odometry maximum speed must be positive")
        self.max_speed = max_speed
        self.samples = deque(maxlen=2000)
        self.epoch = 0

    def add(self, sample):
        if (sample.stamp_ns <= 0 or not sample.frame_id or not sample.child_frame_id
                or not all(math.isfinite(v) for v in sample.pose)):
            raise ValueError("Invalid odometry pose, timestamp or frames")
        reason = None
        if self.samples:
            last = self.samples[-1]
            if sample.stamp_ns < last.stamp_ns:
                reason = "timestamp moved backwards"
            elif (sample.frame_id, sample.child_frame_id) != (
                    last.frame_id, last.child_frame_id):
                reason = "coordinate frames changed"
            elif sample.stamp_ns == last.stamp_ns:
                if any(abs(v) > 1e-9 for v in relative_pose(last.pose, sample.pose)):
                    reason = "odometry changed at duplicate timestamp"
                else:
                    self.samples.pop()
            # Allow 5 cm for timestamp jitter without clipping the odometry.
            elif math.hypot(sample.pose[0] - last.pose[0], sample.pose[1] - last.pose[1]) > (
                    self.max_speed * (sample.stamp_ns - last.stamp_ns) / 1e9 + .05):
                reason = "odometry pose jump"
        if reason:
            self.samples.clear()
            self.epoch += 1
        self.samples.append(sample)
        return reason

    def at(self, stamp_ns):
        match = interpolate_odometry(list(self.samples), stamp_ns, self.max_gap_ns)
        return match[0] if match is not None else None


class OdometryMotionModel:
    """Keep sub-node progress while exposing the original node probabilities.

    Each bin retains a continuous progress mean and variance rather than
    rounding every update back onto a fixed grid. Three-point quadrature and
    moment matching preserve straight-motion mean/variance under subdivision.
    Two overflow bins retain hypotheses before/after the recorded route.
    Heading remains one conditional estimate per bin, not a yaw distribution.
    """
    def __init__(self, path, image_names, noise=0.1):
        if not math.isfinite(noise) or noise < 0:
            raise ValueError("odom_noise must be finite and nonnegative")
        path = Path(path)
        if not path.is_file():
            raise FileNotFoundError("Odometry motion requires " + str(path))
        data = json.loads(path.read_text(encoding="utf-8"))
        if (data.get("schema_version") != 1 or
                data.get("units") != {"position": "m", "angle": "rad"}):
            raise ValueError("Unsupported node_odometry.json schema or units")
        nodes = data["nodes"]
        names = [Path(name).name for name in image_names]
        if not nodes or len(nodes) != len(names) or any(
                node["id"] != i or node["image"] != names[i]
                for i, node in enumerate(nodes)):
            raise ValueError("Odometry nodes do not match the VPR image order")
        poses = np.array([[node["pose"][k] for k in ("x", "y", "yaw")]
                          for node in nodes], dtype=float)
        if not np.isfinite(poses).all():
            raise ValueError("Non-finite topomap pose")
        self.child_frame_id = data.get("child_frame_id")
        if not self.child_frame_id:
            raise ValueError("Topomap child_frame_id is missing")
        distances = np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)
        if np.any(distances <= 0):
            raise ValueError("Consecutive odometry nodes must have distinct positions")
        self.node_s = np.r_[0., np.cumsum(distances)]
        resolution = min(0.1, float(distances.min()) / 2) if len(distances) else 0.1
        count = max(1, int(np.ceil(self.node_s[-1] / resolution)))
        centers = np.linspace(0, self.node_s[-1], count + 1) if len(nodes) > 1 else np.array([0.])
        step = centers[1] - centers[0] if len(centers) > 1 else resolution
        # Two overflow bins retain position beyond the finite recorded route.
        self._bin_centers = np.r_[-step, centers, centers[-1] + step]
        self._bin_midpoints = (centers[:-1] + centers[1:]) / 2
        self._node_midpoints = (self.node_s[:-1] + self.node_s[1:]) / 2
        self._rounded_node_s = np.round(self.node_s, 9)
        self._rounded_node_midpoints = np.round(self._node_midpoints, 9)
        self.grid_s = self._bin_centers.copy()
        self.position_variance = np.zeros(len(self.grid_s))
        self.node_yaw = poses[:, 2].copy()
        self.node_xy = poses[:, :2].copy()
        self._segment_directions = (np.diff(poses[:, :2], axis=0) / distances[:, None]
            if len(nodes) > 1 else np.array([[math.cos(poses[0, 2]), math.sin(poses[0, 2])]]))
        self._update_geometry()
        self.owner_counts = np.bincount(self.owner[1:-1], minlength=len(nodes))
        self.noise = noise
        self.probability = None
        self.heading_offset = np.zeros(len(self.grid_s))
        self.last_odometry = None
        self.last_delta = (0., 0., 0.)
        self.last_boundary_clipped_mass = 0.

    def initialize(self, belief):
        belief = np.asarray(belief, dtype=float)
        self.grid_s = self._bin_centers.copy()
        self.position_variance.fill(0)
        self._update_geometry()
        self.probability = np.zeros(len(self.grid_s))
        owners = self.owner[1:-1]
        self.probability[1:-1] = belief[owners] / self.owner_counts[owners]
        self.probability /= self.probability.sum()
        self.reset_odometry()

    def _update_geometry(self):
        # Coordinate-origin changes can put an exact boundary a few ULPs on
        # either side. Nanometer rounding makes that tie deterministic.
        geometry_s = np.round(self.grid_s, 9)
        self.owner = np.searchsorted(self._rounded_node_midpoints, geometry_s)
        self.yaw = np.interp(self.grid_s, self.node_s, np.unwrap(self.node_yaw))
        segment = np.clip(np.searchsorted(self._rounded_node_s, geometry_s, side='right') - 1,
                          0, len(self._segment_directions) - 1)
        self.tangent = self._segment_directions[segment]

    def interval_weights(self, lower, upper):
        """Conditional mass in a route interval, including sub-cell variance.

        Each occupied bin stores a continuous mean and a Gaussian moment
        approximation. Zero-variance states remain point masses.
        """
        weights = ((self.grid_s >= lower) & (self.grid_s <= upper)).astype(float)
        spread = self.position_variance > 0
        sigma = np.sqrt(self.position_variance[spread])
        means = self.grid_s[spread]
        weights[spread] = ndtr((upper - means) / sigma) - ndtr((lower - means) / sigma)
        return np.clip(weights, 0, 1)

    def reset_odometry(self):
        """Reanchor live input, preserving continuous location and variance."""
        self.last_odometry = None
        self.last_delta = (0., 0., 0.)
        self.last_boundary_clipped_mass = 0.
        self.heading_offset.fill(0)

    def node_belief(self):
        return np.bincount(self.owner, weights=self.probability,
                           minlength=len(self.node_s))

    def progress_probabilities(self, node_idx, tolerance=0.5):
        """Read global posterior mass; do not renormalize a local selection."""
        position = self.node_s[node_idx]
        return {
            'passed': float(self.probability @ self.interval_weights(position, np.inf)),
            'near': float(self.probability @ self.interval_weights(position - tolerance, position + tolerance)),
            'local': float(self.probability[np.abs(self.owner - node_idx) <= 1].sum()),
        }

    def predict(self, sample):
        if sample.child_frame_id != self.child_frame_id:
            raise ValueError("Live odometry child frame differs from topomap body frame")
        previous = self.last_odometry
        if previous is not None and sample == previous:
            return
        self.last_boundary_clipped_mass = 0.
        if previous is not None and (sample.stamp_ns <= previous.stamp_ns or
                (sample.frame_id, sample.child_frame_id) !=
                (previous.frame_id, previous.child_frame_id)):
            self.reset_odometry()
            previous = None
        self.last_odometry = sample
        if previous is None:
            return
        dx, dy, dyaw = relative_pose(previous.pose, sample.pose)
        self.last_delta = (dx, dy, dyaw)
        if math.hypot(dx, dy) < 1e-12:
            self.heading_offset = wrap(self.heading_offset + dyaw)
            return
        active = self.probability > 0
        heading = (self.yaw + self.heading_offset)[active]
        gx, gy = np.cos(heading) * dx - np.sin(heading) * dy, np.sin(heading) * dx + np.cos(heading) * dy
        advance = gx * self.tangent[active, 0] + gy * self.tangent[active, 1]
        absolute_heading = heading + dyaw
        cosine, sine = np.cos(absolute_heading), np.sin(absolute_heading)
        # Independent process variance per travelled meter, rather than a
        # standard deviation per update. Retain uncertainty already in a bin.
        variance = self.position_variance[active] + self.noise**2 * math.hypot(dx, dy)
        sigma = np.sqrt(3 * variance)
        shifts = [(0., 1.)] if not np.any(variance) else [
            (-sigma, 1/6), (0., 2/3), (sigma, 1/6)]
        probability = np.zeros_like(self.probability)
        first_moment, second_moment = np.zeros_like(probability), np.zeros_like(probability)
        cos_sum, sin_sum = np.zeros_like(probability), np.zeros_like(probability)
        for shift, weight in shifts:
            target = self.grid_s[active] + advance + shift
            index = np.searchsorted(self._bin_midpoints, target) + 1
            index[target < 0] = 0
            index[target > self.node_s[-1]] = len(probability) - 1
            mass = self.probability[active] * weight
            # Moments relative to bin centers avoid cancellation on long maps.
            residual = target - self._bin_centers[index]
            probability += np.bincount(index, weights=mass, minlength=len(probability))
            first_moment += np.bincount(index, weights=mass * residual, minlength=len(probability))
            second_moment += np.bincount(index, weights=mass * residual**2, minlength=len(probability))
            cos_sum += np.bincount(index, weights=mass * cosine, minlength=len(probability))
            sin_sum += np.bincount(index, weights=mass * sine, minlength=len(probability))
        mean = np.divide(first_moment, probability, out=np.zeros_like(probability), where=probability > 0)
        second = np.divide(second_moment, probability, out=np.zeros_like(probability), where=probability > 0)
        self.grid_s = self._bin_centers + mean
        self.position_variance = np.maximum(0, second - mean**2)
        self.probability = probability / probability.sum()
        self._update_geometry()
        self.heading_offset = np.where(probability > 0,
            wrap(np.arctan2(sin_sum, cos_sum) - self.yaw), 0.)

    def observe(self, likelihood):
        self.probability *= likelihood[self.owner]
        self.probability /= self.probability.sum()
        return self.node_belief()
