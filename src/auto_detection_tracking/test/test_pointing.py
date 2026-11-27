"""ADT-8: target position -> relative yaw/pitch (CCW positive) and the on-screen direction readout."""
import math

from auto_detection_tracking.pointing import focal_px, orientation_command, readout
from auto_detection_tracking.target import HOLDING, LOST, SEARCHING, TRACKING

SHAPE = (480, 640, 3)
F = focal_px(90, 640)  # 90 deg field of view: the left and right edges are 45 deg off-centre


def box_at(x, y):
    return (x - 5, y - 5, x + 5, y + 5)


def test_target_at_the_centre_needs_no_turn():
    assert orientation_command(box_at(320, 240), SHAPE, F) == (0, 0)


def test_left_is_positive_yaw_and_down_is_positive_pitch():
    yaw, pitch = orientation_command(box_at(0, 240), SHAPE, F)
    assert math.isclose(math.degrees(yaw), 45) and pitch == 0
    yaw, pitch = orientation_command(box_at(320, 480), SHAPE, F)
    assert yaw == 0 and pitch > 0


def test_centred_means_within_5_degrees_of_both_axes_combined():  # DI-7
    assert readout(TRACKING, math.radians(3), math.radians(3), 5) == "CENTRED"  # 4.2 deg off
    assert readout(TRACKING, math.radians(4), math.radians(4), 5) == "MOVE LEFT 4° · DOWN 4°"  # 5.7 deg off


def test_move_names_only_the_axes_that_are_off():
    assert readout(TRACKING, math.radians(-12), 0.0, 5) == "MOVE RIGHT 12°"
    assert readout(HOLDING, math.radians(12), 0.0, 5) == "MOVE LEFT 12° (held)"


def test_lost_gives_direction_only():
    assert readout(LOST, 0.0, math.radians(-25), 5) == "LOST · MOVE UP"
    assert readout(LOST, 0.0, 0.0, 5) == "LOST"


def test_nothing_shown_without_a_target():
    assert readout(SEARCHING, 0.0, 0.0, 5) == ""
