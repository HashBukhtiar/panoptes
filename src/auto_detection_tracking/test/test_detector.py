"""ADT-3: configured class names map to model ids, and a typo fails loudly.
ADT-13: every target belongs to exactly one model group, in key order."""
import pytest

from auto_detection_tracking.detector import class_ids, owners

NAMES = {0: "person", 42: "fork", 43: "knife", 44: "spoon"}


def test_class_names_map_to_ids_in_config_order():
    assert class_ids(NAMES, ["spoon", "fork"]) == [44, 42]


def test_unknown_class_name_raises():
    with pytest.raises(ValueError, match="forceps"):
        class_ids(NAMES, ["fork", "forceps"])


def test_targets_keep_config_order_and_a_shared_class_fails():
    owner = owners([("stand_ins", ["fork", "spoon"]), ("instruments", ["syringe"])])
    assert list(owner) == ["fork", "spoon", "syringe"]  # keys 1, 2, 3
    assert owner["syringe"] == "instruments"
    with pytest.raises(ValueError, match="fork"):
        owners([("stand_ins", ["fork"]), ("instruments", ["fork"])])
