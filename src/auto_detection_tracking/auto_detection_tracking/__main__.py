"""Show the configured camera source in a live window, with detections boxed.

Run from src/auto_detection_tracking/:  python -m auto_detection_tracking [--source SOURCE]
"""
import argparse
from contextlib import closing
from pathlib import Path

import cv2
import yaml

from auto_detection_tracking import detector
from auto_detection_tracking.camera import frames

CONFIG = Path(__file__).resolve().parent.parent / "config" / "auto_detection_tracking.yaml"
WINDOW = "Panoptes - camera active (q to quit)"
GREEN = (0, 255, 0)


def window_open():
    try:
        return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:  # macOS raises instead of returning 0 once the window is gone
        return False


def main():
    parser = argparse.ArgumentParser(description="Show a camera source live with detections boxed.")
    parser.add_argument("--source", help="webcam index or video file path; overrides the config")
    args = parser.parse_args()
    params = yaml.safe_load(CONFIG.read_text())["auto_detection_tracking"]["ros__parameters"]
    source = args.source or params["source"]
    # Load the model before opening the camera, so the camera isn't on while the model loads.
    print(f"Loading {params['model']}...", flush=True)
    detect = detector.load(params["model"], params["classes"], params["min_score"])
    print(f"Showing {source} in a new window (q or Esc to quit)", flush=True)
    # DI-22: the window is the camera-active indicator, so the camera is released before
    # the window closes, and closing the window stops the camera.
    try:
        with closing(frames(source)) as stream:
            for _stamp_ns, image in stream:
                for label, score, (x1, y1, x2, y2) in detect(image):
                    cv2.rectangle(image, (x1, y1), (x2, y2), GREEN, 2)
                    cv2.putText(image, f"{label} {score:.2f}", (x1, y1 - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)
                cv2.imshow(WINDOW, image)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27) or not window_open():
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
