"""Show the configured camera source in a live window, with detections boxed.

Keys 1-6 select the configured classes (stand-ins, then instruments) as the target; 0 clears it.
Only the model that owns the selected class runs (ADT-13). Saying "track <target>" or
"stop tracking" does the same by voice (AI-1); m mutes the microphone.
Run from src/auto_detection_tracking/:
    python -m auto_detection_tracking [--source SOURCE] [--log logs/run.csv]
"""
import argparse
import time
from contextlib import closing
from pathlib import Path

import cv2
import yaml

from auto_detection_tracking import detector
from auto_detection_tracking.camera import frames
from auto_detection_tracking.log import frame_row, open_log
from auto_detection_tracking.pointing import focal_px, orientation_command, readout
from auto_detection_tracking.target import HOLDING, LOST, TRACKING, Tracker, centre
from auto_detection_tracking.voice import Listener

CONFIG = Path(__file__).resolve().parent.parent / "config" / "auto_detection_tracking.yaml"
WINDOW = "Panoptes - camera active (q to quit)"
GREEN, YELLOW, RED, GREY, WHITE = (0, 255, 0), (0, 255, 255), (0, 0, 255), (160, 160, 160), (255, 255, 255)
STATE_COLOUR = {TRACKING: GREEN, HOLDING: YELLOW, LOST: RED}
FONT = cv2.FONT_HERSHEY_SIMPLEX


def window_open():
    try:
        return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:  # macOS raises instead of returning 0 once the window is gone
        return False


def text(image, s, org, colour, scale=0.8):
    cv2.putText(image, s, org, FONT, scale, (0, 0, 0), 5)  # dark outline, readable on any background
    cv2.putText(image, s, org, FONT, scale, colour, 2)


def draw(image, detection, colour, thickness, label):
    _label, _score, (x1, y1, x2, y2) = detection
    cv2.rectangle(image, (x1, y1), (x2, y2), colour, thickness)
    text(image, label, (x1, y1 - 6), colour, 0.6)


def main():
    parser = argparse.ArgumentParser(description="Show a camera source live with detections boxed.")
    parser.add_argument("--source", help="webcam index or video file path; overrides the config")
    parser.add_argument("--log", help="write one CSV row of metadata per frame to this path (never images)")
    args = parser.parse_args()
    params = yaml.safe_load(CONFIG.read_text())["auto_detection_tracking"]["ros__parameters"]
    source = args.source or params["source"]
    groups = {name: params[name] for name in params["detectors"]}
    owner = detector.owners((name, g["classes"]) for name, g in groups.items())
    classes = list(owner)  # every target, in key order
    # Load the models before opening the camera, so the camera isn't on while they load.
    detect = {}
    for name, g in groups.items():
        print(f"Loading {g['model']}...", flush=True)
        try:
            detect[name] = detector.load(g["model"], g["classes"], params["min_score"], g["imgsz"])
        except FileNotFoundError as e:
            raise SystemExit(f"{e}. Trained weights aren't in git: see README 'Models'.") from None
    # Running one model at a time keeps 30 fps; the first group runs until a target from another is picked.
    active = params["detectors"][0]
    listener = None
    if params["voice_model"]:  # an empty string turns voice off
        targets = {spoken: label for g in groups.values() for spoken, label in zip(g["spoken"], g["classes"], strict=True)}
        try:
            listener = Listener(params["voice_model"], targets)
            print("Voice on: say 'track <target>' or 'stop tracking' (m mutes)", flush=True)
        except Exception as e:  # no speech model or no microphone: carry on with the keyboard
            print(f"Voice off ({e!r}); see README 'Voice'. The keyboard still works.", flush=True)
    print(f"Showing {source} in a new window (q or Esc to quit)", flush=True)
    tracker = Tracker(params["lost_after_s"], params["acquire_score"])  # nothing targeted until the operator picks (DI-7)
    log_file, log = open_log(args.log) if args.log else (None, None)
    fps, prev_stamp_ns = 0.0, None
    command, heard = None, None  # (what, when given) for the log; (phrase, when) for the mic line
    # DI-22: the window is the camera-active indicator, so the camera is released before
    # the window closes, and closing the window stops the camera.
    try:
        with closing(frames(source)) as stream:
            for stamp_ns, image in stream:
                while listener and not listener.commands.empty():  # voice commands act like key presses
                    label, phrase, speech_end_ns = listener.commands.get_nowait()
                    tracker.select(label)
                    active = owner.get(label, active)
                    command, heard = (f"voice: {phrase}", speech_end_ns), (phrase, stamp_ns)
                detections = detect[active](image)
                state, target = tracker.update(detections, stamp_ns)
                angles = None
                if target:  # tracking, holding or lost: the command ADT-12 will publish
                    f = focal_px(params["hfov_deg"], image.shape[1])
                    angles = orientation_command(target[2], image.shape, f)  # stamp: stamp_ns
                latency_ms = (time.time_ns() - stamp_ns) / 1e6  # capture -> command ready
                if log:
                    log.writerow(frame_row(stamp_ns, latency_ms, tracker.label, state, target, angles, detections,
                                           command))
                command = None  # logged on the first frame that uses it

                for d in detections:
                    if d is not target:  # detected but not the target (DI-8)
                        draw(image, d, GREY, 1, f"{d[0]} {d[1]:.2f}")
                if state == TRACKING:
                    draw(image, target, GREEN, 3, f"TARGET {target[0]} {target[1]:.2f}")
                elif state == HOLDING:
                    draw(image, target, YELLOW, 3, f"TARGET {target[0]} (held)")
                elif state == LOST:  # mark the last known position
                    x, y = (int(v) for v in centre(target[2]))
                    cv2.drawMarker(image, (x, y), RED, cv2.MARKER_TILTED_CROSS, 24, 3)
                status = (f"Target: {tracker.label} ({classes.index(tracker.label) + 1}) - {state}"
                          if tracker.label else f"Target: none (press 1-{len(classes)})")
                text(image, status, (10, 30), STATE_COLOUR.get(state, YELLOW))
                if angles:
                    text(image, readout(state, *angles, params["centred_deg"]), (10, 65), STATE_COLOUR[state])
                if prev_stamp_ns:  # smoothed so the number is readable
                    fps = 0.9 * fps + 0.1 * 1e9 / max(stamp_ns - prev_stamp_ns, 1)
                prev_stamp_ns = stamp_ns
                perf = f"{fps:.0f} fps  {latency_ms:.0f} ms"
                (width, _), _ = cv2.getTextSize(perf, FONT, 0.8, 2)
                text(image, perf, (image.shape[1] - width - 10, 30), WHITE)
                if listener:  # the microphone indicator (ADT-14 will restyle it)
                    mic = listener.warning or ("MIC MUTED (m)" if listener.muted else "MIC LISTENING (m to mute)")
                    if heard and stamp_ns - heard[1] < 2e9:
                        mic += f'   heard "{heard[0]}"'
                    colour = RED if listener.warning else GREY if listener.muted else WHITE
                    text(image, mic, (10, image.shape[0] - 15), colour, 0.7)

                cv2.imshow(WINDOW, image)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27) or not window_open():
                    break
                if ord("1") <= key < ord("1") + len(classes):
                    tracker.select(classes[key - ord("1")])
                    active = owner[tracker.label]
                    command = (f"key {chr(key)}", time.time_ns())
                elif key == ord("0"):
                    tracker.select(None)
                    command = ("key 0", time.time_ns())
                elif key == ord("m") and listener:
                    listener.muted = not listener.muted
    except KeyboardInterrupt:
        pass
    finally:
        if listener:
            listener.close()
        cv2.destroyAllWindows()
        if log_file:
            log_file.close()
            print(f"Log written to {args.log}", flush=True)


if __name__ == "__main__":
    main()
