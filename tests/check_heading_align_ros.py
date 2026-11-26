#!/usr/bin/env python3
"""Heading alignment on a private master: navigate.py message -> gnm_path_adapter rotation path.

Needs ROS noetic and a built placenav_viz_msgs (HeadingAlign); no robot, TEB, model or LightGlue.
    python3 tests/check_heading_align_ros.py
"""
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
sys.path.insert(0, str(ROOT/'src/placenav'))


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def main():
    port = free_port()
    os.environ.update(ROS_MASTER_URI=f'http://127.0.0.1:{port}', ROS_IP='127.0.0.1')
    os.environ.pop('ROS_HOSTNAME', None)
    children = [subprocess.Popen(['roscore', '-p', str(port)], stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL, start_new_session=True)]
    try:
        time.sleep(3)
        children.append(subprocess.Popen(
            ['rosrun', 'tf2_ros', 'static_transform_publisher', '1', '2', '0', '0.5', '0', '0', 'odom', 'base_link'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True))
        children.append(subprocess.Popen(
            [sys.executable, str(ROOT/'src/placenav/gnm_path_adapter.py'), '--expected-waypoints', '8'],
            start_new_session=True))
        run_checks()
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
    print('heading align ROS check passed')


def run_checks():
    import numpy as np
    import rospy
    from nav_msgs.msg import Path as PathMsg
    from placenav_viz_msgs.msg import HeadingAlign, Viz
    from placenav.heading_align import HeadingAligner
    from placenav.heading_correction import HeadingEstimate
    from utils import waypoints_to_ros_msg
    from navigate import PlaceNavNode

    rospy.init_node('heading_align_check')
    paths = []
    rospy.Subscriber('/placenav/gnm_path', PathMsg, paths.append, queue_size=10)
    viz_pub = rospy.Publisher('/toponav/viz_info', Viz, queue_size=1)
    align_pub = rospy.Publisher('/toponav/heading_align', HeadingAlign, queue_size=1)
    deadline = time.monotonic() + 15
    while viz_pub.get_num_connections() == 0 or align_pub.get_num_connections() == 0:
        assert time.monotonic() < deadline, 'adapter did not subscribe'
        time.sleep(0.1)

    def viz():
        message = Viz()
        message.query_timestamp = rospy.Time.now()
        message.waypoints = waypoints_to_ros_msg(np.column_stack([np.arange(1, 9) * 0.1, np.zeros(8)]))
        viz_pub.publish(message)

    def expect(count, label, timeout=3.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if paths:
                path = paths.pop(0)
                assert len(path.poses) == count, (label, len(path.poses))
                return path
            time.sleep(0.05)
        raise AssertionError(f'{label}: no path')

    def expect_none(label, wait=1.):
        time.sleep(wait)
        assert not paths, (label, [len(p.poses) for p in paths])

    viz()
    expect(9, 'waypoints')

    # navigate.py's measurement wrapper and message from a real aligner (stub node).
    published = []
    estimator = SimpleNamespace(estimate=lambda image, node: HeadingEstimate(math.radians(-60), 0.05, 80),
                                last={'matches': 120, 'inliers': 80})
    node = SimpleNamespace(heading_align_estimator=estimator, heading_align_pub=SimpleNamespace(
        publish=published.append), _heading_align_status=None)
    estimate = PlaceNavNode._measure_heading(node, np.zeros((4, 4, 3), np.uint8), 3)
    assert abs(math.degrees(estimate.dpsi) + 60) < 1e-9
    broken = SimpleNamespace(heading_align_estimator=SimpleNamespace(
        estimate=lambda *a: (_ for _ in ()).throw(RuntimeError('boom')), last={}))
    assert PlaceNavNode._measure_heading(broken, np.zeros((4, 4, 3), np.uint8), 3) is None
    assert PlaceNavNode._measure_heading(node, None, 3) is None

    aligner = HeadingAligner([0., 0., 0., math.radians(60)])
    aligner.new_cycle(lambda n: estimate, 0.5, 10.)
    assert not aligner.gate(3)
    command = aligner.finish_cycle(3)
    odometry = SimpleNamespace(pose=(1.0, 2.0, 0.5), frame_id='odom')
    PlaceNavNode._publish_heading_align(node, rospy.Time.now(), odometry, command, 2, 3)
    message = published[-1]
    assert message.active and message.status == 'ROTATING' and message.node_idx == 3, message
    assert abs(message.target_yaw - (0.5 + math.radians(60))) < 1e-9
    assert (message.robot.x, message.robot.y) == (1.0, 2.0)

    # An active message replaces the waypoint path with an in-place rotation.
    message.header.stamp = rospy.Time.now()
    align_pub.publish(message)
    path = expect(2, 'rotation')
    first, last = path.poses
    assert (first.pose.position.x, first.pose.position.y) == (1.0, 2.0) == (last.pose.position.x, last.pose.position.y)
    target = 2 * math.atan2(last.pose.orientation.z, last.pose.orientation.w)
    assert abs(math.atan2(math.sin(target - message.target_yaw), math.cos(target - message.target_yaw))) < 1e-6
    viz()
    expect_none('waypoints during rotation', 0.5)

    # An inactive message ends it at once.
    message.active, message.status = False, 'ALIGNED'
    align_pub.publish(message)
    expect_none('inactive alignment publishes nothing', 0.3)
    viz()
    expect(9, 'waypoints after alignment')

    # A stale active message (navigate.py stopped publishing) does not block waypoints.
    message.active, message.status = True, 'ROTATING'
    align_pub.publish(message)
    expect(2, 'rotation again')
    time.sleep(1.3)
    viz()
    expect(9, 'waypoints after a stale alignment')


if __name__ == '__main__':
    main()
