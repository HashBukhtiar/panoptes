"""ADT-3: configured class names map to model ids, and a typo fails loudly."""
import pytest

from auto_detection_tracking.detector import class_ids

NAMES = {0: "person", 42: "fork", 43: "knife", 44: "spoon"}


def test_class_names_map_to_ids_in_config_order():
    assert class_ids(NAMES, ["spoon", "fork"]) == [44, 42]


def test_unknown_class_name_raises():
    with pytest.raises(ValueError, match="forceps"):
        class_ids(NAMES, ["fork", "forceps"])
