"""Split navigation between two machines, without ROS.

navigate.py --nav-role localization (parrot) runs VPR, the Bayesian filter, subgoal
selection, the local pose tracker and heading correction, and publishes one
SubgoalCommand per localized image. navigate.py --nav-role waypoint (Xavier) runs only the
waypoint model on its own, newer camera frames, using the latest command.

The two run in parallel: the waypoint node does not wait for the command of its own frame.
The relative subgoal pose is therefore moved from the command's image time to the waypoint
node's image time with odometry, and commands too far from that time are not used.
"""
from dataclasses import dataclass
import math
from typing import Optional, Tuple

from placenav.topomap_odometry import relative_pose

Pose = Tuple[float, float, float]


@dataclass(frozen=True)
class SubgoalCommand:
    stamp_ns: int                # camera image the localization node used
    current_node_idx: int
    subgoal_idx: int
    reached_goal: bool
    goal_pose: Optional[Pose]    # relative subgoal pose at stamp_ns, or None (image only)


def compose(pose, relative):
    """Pose `relative` (given in the frame of `pose`) in the frame `pose` is given in."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    yaw = pose[2] + relative[2]
    return (pose[0] + c * relative[0] - s * relative[1],
            pose[1] + s * relative[0] + c * relative[1],
            math.atan2(math.sin(yaw), math.cos(yaw)))


def move_goal_pose(goal_pose, odom_then, odom_now):
    """Relative goal at `odom_then` -> relative goal at `odom_now` (same odometry frame)."""
    return relative_pose(odom_now, compose(odom_then, goal_pose))


def command_status(command, image_stamp_ns, previous_stamp_ns, map_size, max_age_ns):
    """Why `command` cannot steer the frame at `image_stamp_ns`, or None if it can.

    Stamps of both machines are compared (one clock via chrony, or sim time). The command is
    stale when it is older than the previous camera frame by more than max_age_ns: the
    localization node has stopped following the camera. Measuring from the previous frame
    keeps the first frame after a camera gap, whose own command cannot exist yet; the goal
    pose is still moved over the gap with odometry.
    """
    if command is None:
        return 'no subgoal command yet'
    if not 0 <= command.subgoal_idx < map_size:
        return 'subgoal %d outside the topomap (%d nodes)' % (command.subgoal_idx, map_size)
    if command.stamp_ns - image_stamp_ns > max_age_ns:
        return 'subgoal command is %.2f s newer than the camera frame' % (
            (command.stamp_ns - image_stamp_ns) / 1e9)
    reference = image_stamp_ns if previous_stamp_ns is None else min(previous_stamp_ns, image_stamp_ns)
    if reference - command.stamp_ns > max_age_ns:
        return 'subgoal command is %.2f s older than the previous camera frame' % (
            (reference - command.stamp_ns) / 1e9)
    return None


def valid_goal_pose(values):
    pose = tuple(float(v) for v in values)
    return pose if len(pose) == 3 and all(math.isfinite(v) for v in pose) else None
