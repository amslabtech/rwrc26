#!/usr/bin/env python3
"""Is the parrot -> Xavier link or the inference the bottleneck? Measures how camera
frames arrive over ROS, with no model and no robot motion.

  parrot (inside any ROS environment; answers time pings so Xavier can align clocks):
      python3 link_probe.py echo
  parrot, only if the real camera is not running (raw frames of the camera.yaml size):
      python3 link_probe.py camera [--width 1280 --height 720 --rate 5]
  parrot (baseline, same machine as the camera):
      python3 link_probe.py measure --local
  Xavier:
      /mnt/ssd/PlaceNav/docker/xavier/run_xavier.sh python3 ../docker/xavier/link_probe.py measure

Per frame it records header.stamp (set by the camera driver on parrot), the arrival time
and the size. Latency = arrival - stamp; on the Xavier the parrot clock offset is estimated
from ping/pong with the echo node (no chrony needed). Comparing the Xavier run with the
parrot baseline isolates what the link adds. `--work-ms` replays navigate.py's loop
(take the newest frame when it arrives, or with --trigger timer every 1/rate s as before,
then compute for that long) to show the end-to-end
age of a waypoint for a given inference time (Xavier bench: CosPlace+GNM 42 ms,
CosPlace+OmniVLA-edge 122 ms, SALAD/MegaLoc+GNM ~142 ms).
"""
import argparse
from pathlib import Path
import statistics
import struct
import sys
import threading
import time

import rospy
from std_msgs.msg import Float64MultiArray

PING, PONG = "/link_probe/ping", "/link_probe/pong"
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/"src"/"placenav"))


def check_master():
    """Fail in seconds with the ROS environment instead of rospy's endless retry."""
    import os
    import socket
    import rosgraph
    env = " ".join(f"{k}={os.environ.get(k, '(unset)')}" for k in ("ROS_MASTER_URI", "ROS_IP", "ROS_HOSTNAME"))
    previous = socket.getdefaulttimeout()
    socket.setdefaulttimeout(3)
    try:
        rosgraph.Master("/link_probe").getPid()
    except Exception as error:  # noqa: BLE001 - any failure means the same thing here
        raise SystemExit(f"ROS master unreachable ({error!r}).\n  {env}\n"
                         "  On parrot check `ss -ltn | grep 11311`: nothing = no roscore; "
                         "127.0.0.1:11311 = roscore bound to localhost only (ROS_IP/ROS_HOSTNAME "
                         "was localhost when it started).")
    finally:
        socket.setdefaulttimeout(previous)
    print(f"master ok: {env}")


def camera(args):
    """Stand-in for the camera driver: raw rgb8 frames of the given size, stamped at send time."""
    import os
    from sensor_msgs.msg import Image
    check_master()
    rospy.init_node("link_probe_camera")
    pub = rospy.Publisher(args.topic, Image, queue_size=1)
    data = os.urandom(args.width * args.height * 3)
    print(f"publishing {args.width}x{args.height} rgb8 ({len(data) / 1e6:.2f} MB) "
          f"at {args.rate:g} Hz on {args.topic} (Ctrl-C to stop)")
    rate = rospy.Rate(args.rate)
    while not rospy.is_shutdown():
        msg = Image(height=args.height, width=args.width, encoding="rgb8", step=args.width * 3, data=data)
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()


def echo(_args):
    check_master()
    rospy.init_node("link_probe_echo")
    pub = rospy.Publisher(PONG, Float64MultiArray, queue_size=10, tcp_nodelay=True)

    def reply(msg):
        pub.publish(Float64MultiArray(data=list(msg.data) + [time.time()]))

    rospy.Subscriber(PING, Float64MultiArray, reply, queue_size=10, tcp_nodelay=True)
    print("echo ready (Ctrl-C to stop)")
    rospy.spin()


def clock_offset(n=40, timeout=5.0):
    """(parrot clock - this clock, best round trip) in s, or (None, None) without an echo node."""
    pongs = []
    sub = rospy.Subscriber(PONG, Float64MultiArray, lambda m: pongs.append((time.time(), m.data)),
                           queue_size=50, tcp_nodelay=True)
    pub = rospy.Publisher(PING, Float64MultiArray, queue_size=50, tcp_nodelay=True)
    deadline = time.time() + timeout
    while (pub.get_num_connections() == 0 or sub.get_num_connections() == 0) and time.time() < deadline:
        time.sleep(0.05)
    if pub.get_num_connections() == 0:
        sub.unregister(), pub.unregister()
        return None, None
    time.sleep(0.3)
    for seq in range(n):
        pub.publish(Float64MultiArray(data=[seq, time.time()]))
        time.sleep(0.05)
    time.sleep(0.5)
    sub.unregister(), pub.unregister()
    samples = sorted((t1 - d[1], d[2] - (d[1] + t1) / 2) for t1, d in pongs)  # (rtt, offset)
    if not samples:
        return None, None
    best = samples[:max(3, len(samples) // 4)]  # least queued round trips
    return statistics.median(o for _, o in best), best[0][0]


def stats_ms(xs):
    xs = sorted(xs)
    if not xs:
        return "n/a"
    p90 = xs[min(len(xs) - 1, int(0.9 * len(xs)))]
    return f"median {1e3 * statistics.median(xs):7.1f}  p90 {1e3 * p90:7.1f}  max {1e3 * xs[-1]:7.1f}"


def measure(args):
    check_master()
    rospy.init_node("link_probe", anonymous=True)
    if args.local:
        offset, rtt = 0.0, None
        print("clock: same machine as the camera (offset 0)")
    else:
        offset, rtt = clock_offset()
        t_offset = time.time()
        if offset is None:
            print("clock: no echo node -> latency not shown (run `link_probe.py echo` on parrot)")
        else:
            print(f"clock: parrot - here = {1e3 * offset:+.1f} ms (best ROS round trip {1e3 * rtt:.1f} ms)")

    frames, lock, newest = [], threading.Lock(), [None]
    from frame_gate import FrameGate  # the one navigate.py uses
    gate = FrameGate(args.rate, is_shutdown=rospy.is_shutdown)

    def on_frame(msg):  # AnyMsg: no deserialisation cost, any type that starts with a Header
        t = time.time()
        _seq, sec, nsec = struct.unpack_from("<3I", msg._buff, 0)
        rec = (t, sec + 1e-9 * nsec, len(msg._buff))
        with lock:
            frames.append(rec)
            newest[0] = rec
        gate.notify()

    # Same subscriber settings as navigate.py unless --buff-size is given (rospy default 64 KiB).
    rospy.Subscriber(args.topic, rospy.AnyMsg, on_frame, queue_size=1,
                     buff_size=args.buff_size)
    t_wait = time.time()
    while not frames and time.time() - t_wait < 10 and not rospy.is_shutdown():
        time.sleep(0.1)
    if not frames:
        print(f"no message on {args.topic} in 10 s. Is the camera running? From the Xavier, the "
              "publisher's host name must resolve: `getent hosts parrot`, or start the camera with "
              "ROS_IP=192.168.1.146 on parrot.")
        return
    print(f"receiving {args.topic}; measuring {args.duration:.0f} s"
          + (f" with a {args.work_ms:.0f} ms compute loop ({args.trigger}-driven, {args.rate:g} Hz)"
             if args.work_ms else ""))

    outputs, t_end, used = [], time.time() + args.duration, None
    period = 1.0 / args.rate
    while time.time() < t_end and not rospy.is_shutdown():
        tick = time.time()
        if args.work_ms and args.trigger == "frame":
            gate.sleep()  # navigate.py now: start when a frame arrives
        if args.work_ms:
            with lock:
                rec = newest[0]
            if rec is not None and rec is not used:
                used = rec
                time.sleep(args.work_ms / 1e3)  # stands in for VPR + waypoint inference
                outputs.append((time.time(), rec[1]))
        if args.trigger == "timer" or not args.work_ms:  # navigate.py before: rospy.Rate
            time.sleep(max(0.0, period - (time.time() - tick)))

    with lock:
        fr = list(frames)
    clock_of = None  # parrot - here at a given local time
    if args.local:
        clock_of = lambda t: 0.0  # noqa: E731
    elif offset is not None:
        # Re-measure: a clock that chrony is still slewing would otherwise bias every latency.
        offset2, rtt2 = clock_offset()
        if offset2 is None:
            clock_of = lambda t: offset  # noqa: E731
        else:
            t_offset2 = time.time()
            rate_ = (offset2 - offset) / (t_offset2 - t_offset)
            clock_of = lambda t: offset + rate_ * (t - t_offset)  # noqa: E731
            print(f"clock: parrot - here = {1e3 * offset2:+.1f} ms at the end (round trip {1e3 * rtt2:.1f} ms); "
                  f"drift {1e3 * (offset2 - offset):+.1f} ms over {t_offset2 - t_offset:.0f} s "
                  f"= {1e6 * rate_:+.0f} ppm (synced clocks: |drift| < 1 ms)")
    n = len(fr)
    if n < 3:
        print(f"only {n} frames received")
        return
    arrive = [f[0] for f in fr]
    stamps = [f[1] for f in fr]
    size = statistics.median(f[2] for f in fr)
    span = arrive[-1] - arrive[0]
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    src_period = statistics.median(gaps)
    lost = sum(max(0, round(g / src_period) - 1) for g in gaps)

    print()
    print(f"frame size        {size / 1e6:6.2f} MB  (at 100 Mbps ~{size * 8 / 94e6 * 1e3:.0f} ms, "
          f"at 1 Gbps ~{size * 8 / 940e6 * 1e3:.0f} ms on the wire)")
    print(f"camera rate       {1 / src_period:6.2f} Hz (from header stamps)")
    print(f"received          {(n - 1) / span:6.2f} Hz, {n} frames, {lost} skipped "
          f"({100 * lost / (n + lost):.0f}%), {sum(f[2] for f in fr) / span / 1e6:.2f} MB/s")
    if clock_of is not None:
        lat = [a - (s - clock_of(a)) for a, s in zip(arrive, stamps)]
        k = max(1, n // 5)
        print(f"stamp->arrival ms {stats_ms(lat)}")
        print(f"  trend           first {k}: {1e3 * statistics.median(lat[:k]):.1f} ms, "
              f"last {k}: {1e3 * statistics.median(lat[-k:]):.1f} ms (growing = backlog; clock drift corrected)")
        if outputs:
            ages = [t - (s - clock_of(t)) for t, s in outputs]
            print(f"waypoint age ms   {stats_ms(ages)}  ({len(outputs) / args.duration:.2f} outputs/s; "
                  f"stamp -> end of {args.work_ms:.0f} ms compute)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)
    sub.add_parser("echo", help="on parrot: answer time pings")
    c = sub.add_parser("camera", help="on parrot, when the real camera is not running: publish fake frames")
    c.add_argument("--topic", default="/usb_cam/image_raw")
    c.add_argument("--width", type=int, default=1280)
    c.add_argument("--height", type=int, default=720)
    c.add_argument("--rate", type=float, default=5.0)
    m = sub.add_parser("measure", help="measure frame arrival on this machine")
    m.add_argument("--topic", default="/usb_cam/image_raw")
    m.add_argument("--duration", type=float, default=30.0)
    m.add_argument("--local", action="store_true", help="running on the camera's machine (no clock offset)")
    m.add_argument("--work-ms", type=float, default=0.0, help="simulated inference time per cycle")
    m.add_argument("--rate", type=float, default=5.0, help="navigate.py loop rate (robots.yaml frame_rate)")
    m.add_argument("--trigger", choices=("frame", "timer"), default="frame",
                   help="compute loop start: on frame arrival (navigate.py now) or free-running timer (before)")
    m.add_argument("--buff-size", type=int, default=65536, help="rospy subscriber buffer (navigate.py: default)")
    args = ap.parse_args()
    {"echo": echo, "camera": camera, "measure": measure}[args.mode](args)


if __name__ == "__main__":
    main()
