"""ADT-14: the operator view draws every state without failing, on frames of the camera's size."""
import math

import numpy as np
import pytest

from auto_detection_tracking.target import HOLDING, LOST, SEARCHING, TRACKING
from auto_detection_tracking.ui import pretty, render, ring_px

GROUPS = [("stand_ins", ["fork", "spoon", "knife"]), ("instruments", ["syringe", "adson_forceps", "needle_driver"])]
FORCEPS = ("adson_forceps", 0.81, (500, 150, 690, 240))
SYRINGE = ("syringe", 0.45, (400, 220, 580, 490))


def test_class_names_read_naturally():
    assert pretty("adson_forceps") == "Adson forceps"
    assert pretty("needle_driver") == "Needle driver"


def test_ring_radius_is_the_pixel_offset_of_that_angle():
    assert ring_px(45, 1000) == 1000
    assert ring_px(5, 1259) == 110  # the DI-7 ring on the MacBook webcam (53.9 deg wide, 1280 px)


@pytest.mark.parametrize("selected, state, target, angles, mic", [
    (None, None, None, None, None),  # nothing selected, voice off
    ("adson_forceps", SEARCHING, None, None, {"muted": False, "warning": None, "heard": "track forceps"}),
    ("adson_forceps", TRACKING, FORCEPS, (math.radians(3), math.radians(-7)), {"muted": True, "warning": None, "heard": None}),
    ("adson_forceps", TRACKING, FORCEPS, (math.radians(1), math.radians(1)), None),  # centred
    ("adson_forceps", HOLDING, FORCEPS, (math.radians(3), math.radians(-7)), None),
    ("adson_forceps", LOST, ("adson_forceps", 0.81, (0, 0, 40, 30)), (math.radians(25), math.radians(-14)),
     {"muted": False, "warning": "mic is silent: check microphone permission", "heard": None}),  # target at the edge
])
def test_every_state_draws_on_the_frame_without_changing_its_size(selected, state, target, angles, mic):
    image = np.zeros((720, 1280, 3), np.uint8)
    detections = [SYRINGE] + ([target] if state == TRACKING else [])
    render(image, groups=GROUPS, selected=selected, state=state, target=target, detections=detections,
           angles=angles, f=1259.0, centred_deg=5.0, fps=30.0, latency_ms=23.0, mic=mic)
    assert image.shape == (720, 1280, 3) and image.any()
