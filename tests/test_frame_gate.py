"""FrameGate: inference cycles follow camera frames, capped near frame_rate."""
import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'/'placenav'))
from frame_gate import FrameGate  # noqa: E402


class FrameGateTest(unittest.TestCase):
    def test_returns_right_after_a_frame_instead_of_a_timer_tick(self):
        gate = FrameGate(frame_rate=5)
        gate.notify(); gate.sleep()  # first cycle
        threading.Timer(0.25, gate.notify).start()  # next frame 250 ms later
        t0 = time.monotonic(); gate.sleep(); waited = time.monotonic()-t0
        self.assertAlmostEqual(waited, 0.25, delta=0.03)

    def test_frame_queued_during_inference_starts_next_cycle_at_once(self):
        gate = FrameGate(frame_rate=5)
        gate.notify(); gate.sleep()
        time.sleep(0.19); gate.notify()  # arrived while "inference" ran
        t0 = time.monotonic(); gate.sleep()
        self.assertLess(time.monotonic()-t0, 0.03)

    def test_faster_camera_is_capped_near_frame_rate(self):
        gate = FrameGate(frame_rate=5)
        stop = threading.Event()
        def camera():  # 30 Hz
            while not stop.is_set():
                gate.notify(); time.sleep(1/30)
        threading.Thread(target=camera, daemon=True).start()
        t0 = time.monotonic()
        for _ in range(6):
            gate.sleep()
        stop.set()
        self.assertGreaterEqual(time.monotonic()-t0, 5*0.9/5-0.01)

    def test_shutdown_ends_wait_without_frames(self):
        gate = FrameGate(frame_rate=5, is_shutdown=lambda: True)
        t0 = time.monotonic(); gate.sleep()
        self.assertLess(time.monotonic()-t0, 0.7)


if __name__ == '__main__':
    unittest.main()
