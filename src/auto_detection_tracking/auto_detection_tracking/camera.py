"""Camera source: a webcam or a video file, read one timestamped frame at a time."""
import time

import cv2


def frames(source):
    """Yield (stamp_ns, image) until the source ends or a read fails.

    source: a webcam index ("0", or the int 0) or a path to a video file.
    stamp_ns: wall-clock time in nanoseconds (the clock ROS 2 header stamps use),
    taken right after the grab and before decoding.
    """
    source = str(source)
    is_file = not source.isdigit()
    cap = cv2.VideoCapture(source if is_file else int(source))
    if not cap.isOpened():
        raise RuntimeError(
            f"Could not open camera source {source!r}. For a webcam, check the index and that "
            "this terminal has camera permission (System Settings > Privacy & Security > Camera). "
            "For a video file, check the path.")
    try:
        if is_file:  # play at the file's own frame rate so it behaves like a live camera
            fps = cap.get(cv2.CAP_PROP_FPS)
            period = 1.0 / fps if fps > 0 else 0.0
            next_due = time.monotonic()
        while True:
            if is_file:
                delay = next_due - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                next_due = max(next_due, time.monotonic()) + period
            # A webcam needs no pacing: grab() waits for its next frame.
            if not cap.grab():
                return
            # ponytail: host time when the frame arrived, not exposure time; use the ZED SDK's
            # image timestamp once camera-driver exists.
            stamp_ns = time.time_ns()
            ok, image = cap.retrieve()
            if not ok:
                return
            yield stamp_ns, image
    finally:
        cap.release()
