"""Per-frame metadata log (ADT-9), for measuring DI-7, DI-8 and DI-9. Never image data (DI-22)."""
import csv
import json
import math
from pathlib import Path

from auto_detection_tracking.pointing import off_centre

FIELDS = ["stamp_ns", "latency_ms", "selected", "state", "target_label", "target_score",
          "target_box", "yaw_deg", "pitch_deg", "off_centre_deg", "detections", "command", "command_ns"]


def open_log(path):
    """Create the CSV (and its folder) with a header row; return (file, csv.DictWriter)."""
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    file = open(path, "w", newline="")
    writer = csv.DictWriter(file, FIELDS)
    writer.writeheader()
    return file, writer


def frame_row(stamp_ns, latency_ms, selected, state, target, angles, detections, command=None):
    """One row. target is (label, score, box) or None; angles is (yaw, pitch) in radians or None.
    command is (what, given_ns) on the first frame after a selection command, e.g. ("key 5", ns) or
    ("voice: track forceps", ns), where given_ns is the key press or the end of speech (DI-7 timing).

    Columns left out of the dict are written blank.
    """
    row = {"stamp_ns": stamp_ns, "latency_ms": round(latency_ms, 1), "selected": selected or "",
           "state": state or "",
           "detections": json.dumps([[label, round(score, 3), list(box)] for label, score, box in detections])}
    if target:
        row.update(target_label=target[0], target_score=round(target[1], 3),
                   target_box=" ".join(str(v) for v in target[2]))
    if angles:
        yaw, pitch = angles
        row.update(yaw_deg=round(math.degrees(yaw), 2), pitch_deg=round(math.degrees(pitch), 2),
                   off_centre_deg=round(math.degrees(off_centre(yaw, pitch)), 2))
    if command:
        row.update(command=command[0], command_ns=command[1])
    return row
