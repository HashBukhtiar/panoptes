"""ADT-4/5: public datasets merge into our classes, polygons become boxes, other classes become background."""
import pytest
import yaml

from auto_detection_tracking.dataset import merge, remap, to_box


def test_polygon_becomes_its_bounding_box_clipped_to_the_image():
    assert to_box([0.1, 0.2, 0.3, 0.6, 0.2, 0.4]) == pytest.approx((0.2, 0.4, 0.2, 0.4))
    assert to_box([-0.1, 0.2, 0.3, 0.2, 0.3, 0.6]) == pytest.approx((0.15, 0.4, 0.3, 0.4))  # clipped at x = 0


def test_a_plain_box_passes_through():
    assert to_box([0.5, 0.5, 0.2, 0.4]) == (0.5, 0.5, 0.2, 0.4)


def test_remap_keeps_mapped_classes_only_and_renumbers_them():
    text = "1 0.1 0.1 0.3 0.1 0.3 0.5\n0 0.5 0.5 0.2 0.2\n2 0.6 0.6 0.1 0.1\n"
    assert remap(text, {1: 0, 2: 2}).splitlines() == [
        "0 0.200000 0.300000 0.200000 0.400000",
        "2 0.600000 0.600000 0.100000 0.100000",
    ]
    assert remap("", {0: 0}) == ""


def test_merge_links_images_and_writes_data_yaml(tmp_path):
    src = tmp_path / "mj"
    (src / "train" / "images").mkdir(parents=True)
    (src / "train" / "labels").mkdir(parents=True)
    (src / "data.yaml").write_text(yaml.safe_dump({"names": ["Clamp", "Forcep", "Needle_holder"]}))
    (src / "train" / "images" / "a.jpg").write_bytes(b"jpg")
    (src / "train" / "labels" / "a.txt").write_text("0 0.5 0.5 0.1 0.1\n2 0.4 0.4 0.2 0.2\n")
    out = tmp_path / "merged"

    merge(out, ["syringe", "adson_forceps", "needle_driver"],
          [("mj_", src, {"Forcep": "adson_forceps", "Needle_holder": "needle_driver"})])

    assert (out / "images" / "train" / "mj_a.jpg").is_symlink()
    assert (out / "labels" / "train" / "mj_a.txt").read_text() == "2 0.400000 0.400000 0.200000 0.200000\n"
    assert yaml.safe_load((out / "data.yaml").read_text())["names"] == {0: "syringe", 1: "adson_forceps", 2: "needle_driver"}
