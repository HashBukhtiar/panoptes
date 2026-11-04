"""Show the configured camera source in a live window.

Run from src/auto_detection_tracking/:  python -m auto_detection_tracking
"""
from contextlib import closing
from pathlib import Path

import cv2
import yaml

from auto_detection_tracking.camera import frames

CONFIG = Path(__file__).resolve().parent.parent / "config" / "auto_detection_tracking.yaml"
WINDOW = "Panoptes - camera active (q to quit)"


def window_open():
    try:
        return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:  # macOS raises instead of returning 0 once the window is gone
        return False


def main():
    params = yaml.safe_load(CONFIG.read_text())["auto_detection_tracking"]["ros__parameters"]
    # DI-22: the window is the camera-active indicator, so the camera is released before
    # the window closes, and closing the window stops the camera.
    try:
        with closing(frames(params["source"])) as stream:
            for _stamp_ns, image in stream:
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
