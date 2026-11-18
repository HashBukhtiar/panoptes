"""ADT-9: one metadata row per frame, readable back from the CSV; no image data (DI-22)."""
import csv
import json
import math

from auto_detection_tracking.log import FIELDS, frame_row, open_log

FORK = ("fork", 0.8123, (10, 20, 30, 40))
KNIFE = ("knife", 0.4, (50, 20, 70, 40))


def test_row_with_a_target_fills_every_column():
    row = frame_row(123, 45.67, "fork", "tracking", FORK, (math.radians(3), math.radians(-4)), [FORK, KNIFE])
    assert set(row) == set(FIELDS)
    assert row["target_box"] == "10 20 30 40" and row["target_score"] == 0.812
    assert row["yaw_deg"] == 3.0 and row["pitch_deg"] == -4.0 and row["off_centre_deg"] == 4.99
    assert json.loads(row["detections"]) == [["fork", 0.812, [10, 20, 30, 40]], ["knife", 0.4, [50, 20, 70, 40]]]


def test_row_with_nothing_selected_leaves_the_target_columns_out():
    row = frame_row(123, 10.0, None, None, None, None, [KNIFE])
    assert row["selected"] == "" and row["state"] == ""
    assert not {"target_label", "yaw_deg", "off_centre_deg"} & set(row)


def test_log_file_reads_back_with_header_and_blank_missing_columns(tmp_path):
    path = tmp_path / "logs" / "run.csv"
    file, writer = open_log(path)
    writer.writerow(frame_row(1, 5.0, "fork", "searching", None, None, []))
    file.close()
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    assert list(rows[0]) == FIELDS
    assert rows[0]["state"] == "searching" and rows[0]["target_label"] == ""
