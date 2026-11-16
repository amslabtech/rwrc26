"""Start each inference cycle when a new camera frame arrives, not on a free-running timer.

navigate.py used rospy.Rate(frame_rate): camera and timer both run at 5 Hz but are not
aligned, so a frame waited 0-200 ms (depending on their phase) before inference began.
FrameGate.sleep() is a drop-in for that rate.sleep(): it returns as soon as a frame newer
than the previous cycle is queued. The cycle rate stays capped near frame_rate, so a faster
camera does not make the Bayesian filter step more often than before.
"""
import threading
import time


class FrameGate:
    def __init__(self, frame_rate, min_period_fraction=0.9, is_shutdown=lambda: False):
        self._new_frame = threading.Event()
        self._min_period = min_period_fraction / float(frame_rate)
        self._is_shutdown = is_shutdown
        self._last_cycle = None

    def notify(self):
        """Called from the camera callback after the frame is queued."""
        self._new_frame.set()

    def sleep(self):
        if self._last_cycle is not None:
            remaining = self._last_cycle + self._min_period - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
        # Short waits so that shutdown is noticed while no camera frame arrives.
        while not self._new_frame.wait(0.5):
            if self._is_shutdown():
                return
        # Clear before the caller copies the queue: a frame queued after this point
        # sets the event again and starts the next cycle without waiting.
        self._new_frame.clear()
        self._last_cycle = time.monotonic()
