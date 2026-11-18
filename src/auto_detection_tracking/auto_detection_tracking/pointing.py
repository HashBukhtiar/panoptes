"""Direction command (ADT-8): turn the target's position in the image into camera angles.

Angles follow CameraOrientationCommand: radians, counter-clockwise positive about REP 103 axes
(x forward, y left, z up), so +yaw turns the camera left and +pitch tilts it down.
"""
import math

from auto_detection_tracking.target import HOLDING, LOST, TRACKING, centre


def focal_px(hfov_deg, width):
    """Focal length in pixels of a pinhole camera with this horizontal field of view.

    The ZED SDK reports its calibrated focal length directly; use that instead once it's here.
    """
    return (width / 2) / math.tan(math.radians(hfov_deg) / 2)


def orientation_command(box, image_shape, f):
    """Relative (yaw, pitch) in radians that would centre the box: is_absolute=False, roll=0."""
    height, width = image_shape[:2]
    x, y = centre(box)
    return math.atan((width / 2 - x) / f), math.atan((y - height / 2) / f)


def off_centre(yaw, pitch):
    """Angle between the camera's straight-ahead direction and the line to the target."""
    return math.atan(math.hypot(math.tan(yaw), math.tan(pitch)))


def readout(state, yaw, pitch, centred_deg):
    """On-screen stand-in for the command while there's no mechanism to move ("" if no target)."""
    if state not in (TRACKING, HOLDING, LOST):
        return ""
    axes = [("LEFT" if yaw > 0 else "RIGHT", math.degrees(abs(yaw))),
            ("DOWN" if pitch > 0 else "UP", math.degrees(abs(pitch)))]
    if state == LOST:  # the box is the last known position: only its direction means anything
        words = [word for word, deg in axes if deg > centred_deg]
        return " ".join(["LOST - MOVE", *words]) if words else "LOST"
    if math.degrees(off_centre(yaw, pitch)) <= centred_deg:  # DI-7
        text = "CENTRED"
    else:  # OpenCV's built-in fonts have no degree sign, hence "deg"
        text = "MOVE " + " ".join(f"{word} {deg:.0f} deg" for word, deg in axes if round(deg) >= 1)
    return text + (" (held)" if state == HOLDING else "")
