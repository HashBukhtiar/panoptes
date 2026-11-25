"""Score a detector on clips whose contents are known, without labelling any frames.

Each clip lists the classes physically in it for the whole clip. Per clip it reports:
- detected: % of frames where each class is boxed. For a class that IS in the clip that's the
  detection rate; for one that ISN'T, every box is a confusion (DI-8).
- tracked / first lock: selecting each class from the first frame, as the app would. For a class
  in the clip: % of frames tracking it and seconds until the first lock (DI-7 acquisition).
  For a class not in the clip it should never lock.
Then a summary over all clips with the same contents: how many takes locked each class within 3 s
(DI-7) and how many ever locked it (should be 0 for a class not in them, DI-8).
Frames are processed in memory only; nothing is saved (DI-22).

Run from src/auto_detection_tracking/:
    python scripts/score_clips.py --model PATH/best.pt --classes syringe adson_forceps needle_driver \
        --clip forceps.mov adson_forceps --clip all.mov syringe,adson_forceps,needle_driver
"""
import argparse
from collections import Counter, defaultdict

import cv2
import yaml

from auto_detection_tracking import detector
from auto_detection_tracking.__main__ import CONFIG
from auto_detection_tracking.target import TRACKING, Tracker


def score(detect, clip, present, classes, params):
    """Print one clip's table; return (frames, detected, tracked, first_lock)."""
    cap = cv2.VideoCapture(clip)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    trackers = {c: Tracker(params["lost_after_s"], params["acquire_score"]) for c in classes}
    for c, t in trackers.items():
        t.select(c)
    frames, detected, tracked, first_lock = 0, dict.fromkeys(classes, 0), dict.fromkeys(classes, 0), {}
    while True:
        ok, image = cap.read()
        if not ok:
            break
        stamp_ns = int(frames / fps * 1e9)  # clip time, so results don't depend on processing speed
        detections = detect(image)
        for c in classes:
            detected[c] += any(d[0] == c for d in detections)
            if trackers[c].update(detections, stamp_ns)[0] == TRACKING:
                tracked[c] += 1
                first_lock.setdefault(c, frames / fps)
        frames += 1
    cap.release()
    print(f"\n### {clip}  ({frames} frames; in clip: {', '.join(present)})\n")
    print("| class | in clip | detected % | tracked % | first lock (s) |\n|---|---|---|---|---|")
    for c in classes:
        lock = f"{first_lock[c]:.1f}" if c in first_lock else "never"
        print(f"| {c} | {'yes' if c in present else 'NO'} | {detected[c] / frames * 100:.0f} | "
              f"{tracked[c] / frames * 100:.0f} | {lock} |")
    return frames, detected, tracked, first_lock


def summarise(groups, classes):
    print("\n## Summary by clip contents\n")
    print("| in clips | takes | class | in clip | detected % | tracked % | locked in <=3 s | ever locked |")
    print("|---|---|---|---|---|---|---|---|")
    for present, g in groups.items():
        for c in classes:
            print(f"| {present} | {g['takes']} | {c} | {'yes' if c in present.split(',') else 'NO'} | "
                  f"{g['detected'][c] / g['frames'] * 100:.0f} | {g['tracked'][c] / g['frames'] * 100:.0f} | "
                  f"{g['fast'][c]}/{g['takes']} | {g['locked'][c]}/{g['takes']} |")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Score a detector on clips with known contents.")
    parser.add_argument("--model", required=True)
    parser.add_argument("--classes", nargs="+", required=True)
    parser.add_argument("--imgsz", type=int, default=640, help="model input size (default 640)")
    parser.add_argument("--clip", nargs=2, action="append", required=True, metavar=("PATH", "PRESENT"),
                        help="video path and comma-separated classes in it for the whole clip")
    args = parser.parse_args()
    params = yaml.safe_load(CONFIG.read_text())["auto_detection_tracking"]["ros__parameters"]
    detect = detector.load(args.model, args.classes, params["min_score"], args.imgsz)
    groups = defaultdict(lambda: {"takes": 0, "frames": 0, "detected": Counter(), "tracked": Counter(),
                                  "fast": Counter(), "locked": Counter()})
    for path, present in args.clip:
        frames, detected, tracked, first_lock = score(detect, path, present.split(","), args.classes, params)
        g = groups[present]
        g["takes"] += 1
        g["frames"] += frames
        g["detected"].update(detected)
        g["tracked"].update(tracked)
        g["fast"].update(c for c, t in first_lock.items() if t <= 3.0)  # DI-7: acquire within 3 s
        g["locked"].update(first_lock.keys())  # count takes, not lock times
    summarise(groups, args.classes)
