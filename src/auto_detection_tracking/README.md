# auto_detection_tracking

Autonomous-mode vision: find the operator's selected instrument and keep it centred. So far it opens a camera source, shows it live (ADT-1), and boxes the configured objects with a pretrained YOLO model (ADT-3), lets you pick one as the target (ADT-6), follows it between frames (ADT-7), and shows the direction the camera should turn (ADT-8). It has six targets from two models (ADT-13): stand-ins (fork, spoon, knife) and the real instruments (syringe, Adson forceps, needle driver). See [Models](#models).

## Setup (once, from the repo root)

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r src/auto_detection_tracking/requirements.txt
```

## Run (every new terminal, from the repo root)

```bash
source .venv/bin/activate
cd src/auto_detection_tracking
python -m auto_detection_tracking                                 # webcam
python -m auto_detection_tracking --source path/to/clip.mov       # or a video file
```

Your prompt shows `(.venv)` once the environment is active. `ModuleNotFoundError: No module named 'cv2'` means it isn't: run the `source` line again. Already in `src/auto_detection_tracking`? Use `source ../../.venv/bin/activate` instead.

Pick the source in `config/auto_detection_tracking.yaml`: a webcam index (`"0"`) or a path to a video file. To try a different source without editing the file, pass `--source`, e.g. `python -m auto_detection_tracking --source path/to/clip.mov`. Relative paths are resolved from the folder you run in. Video files play at their own frame rate. The same file sets the models, which labels each one boxes (`classes`), and the minimum score.

Press `1` to `6` to make a class the target (in the order listed under `detectors`: fork, spoon, knife, syringe, adson_forceps, needle_driver), and `0` to clear it. Only the model that owns the selected target runs, which keeps the video at 30 fps; picking a target from the other group switches models. The target gets a thick green box; other detected objects get thin grey boxes and are ignored (DI-8). The status line shows the tracking state:

- **searching**: not yet seen with a score of at least `acquire_score` since you selected it. Requiring a confident box to start keeps weak look-alikes from being picked up (DI-8).
- **tracking** (green box): detected this frame. Once found, the target follows the box nearest its last position, so a higher-scoring look-alike elsewhere doesn't steal it.
- **holding** (yellow box): missed for less than `lost_after_s`; the last box is kept.
- **lost** (red cross at the last known position): missed for longer. The next detection of the class scoring at least `acquire_score`, anywhere, reacquires it.

Under the status, a second line stands in for the camera command until there's a mechanism to move (ADT-8): `MOVE LEFT 12 deg DOWN 4 deg`, `CENTRED` within `centred_deg` (5°, DI-7), or `LOST - MOVE UP` from the last known position. Directions are from the camera's point of view, so facing your webcam, "LEFT" is your right. The degrees come from `hfov_deg`, the camera's horizontal field of view, set for Hashim's MacBook webcam. For another camera, hold a ruler level at distance D from it, read the width W visible across the frame, and set `hfov_deg` to 2·atan(W / 2D) in degrees. Keep macOS Center Stage off (Control Center > Video Effects): it crops and pans the image, so the angles would be wrong.

Quit with `q`, `Esc`, or by closing the window. The window is the camera-active indicator (DI-22): the camera is open only while the window is showing, and nothing is recorded.

On macOS, the first webcam run asks for camera permission for your terminal app and exits with "Could not open camera source" while the prompt is up. Allow it, then run again. If no prompt appears, turn your terminal app on under System Settings > Privacy & Security > Camera and restart the terminal. If index 0 opens the wrong camera (e.g. an iPhone through Continuity Camera), try `"1"`.

## Models

| Group | Keys | Weights | Input size | Classes |
|---|---|---|---|---|
| `stand_ins` | 1-3 | `yolo26n.pt`: YOLO26n pretrained on COCO, downloaded on first run | 640 | fork, spoon, knife |
| `instruments` | 4-6 | `instruments-public-v1.pt`: YOLO26n fine-tuned on public data (ADT-5) | 1280 | syringe, adson_forceps, needle_driver |

Both files live in `src/auto_detection_tracking/` and are gitignored (`*.pt`). The instrument weights aren't downloadable: copy `instruments-public-v1.pt` from the team's Teams files into that folder. Without it the app stops with "No such file or directory: 'instruments-public-v1.pt'".

**How good the instrument model is:** it has seen public photos only, none of ours. On 26 labelled webcam frames of our instruments at 60-90 cm (score threshold 0.25, input 1280) it found 5 of 10 syringes, 10 of 15 forceps and 0 of 14 needle drivers, and once called a syringe forceps. The public photos are close-ups (an instrument fills about 40% of the image, against 10-18% in ours), and their needle holders look different from ours. Training on our own photos waits for the ZED camera.

**Training data (CC BY 4.0):** [surgical_tool_detection_6class](https://universe.roboflow.com/mj-4iyes/surgical_tool_detection_6class) by mj-4iyes (its Forcep and Needle_holder classes) and [syringe](https://universe.roboflow.com/syringe-project/syringe-tovbf) by syringe-project, both on Roboflow Universe. Changes: other classes dropped, polygon outlines converted to boxes, merged with `python -m auto_detection_tracking.dataset`, then trained with `yolo detect train model=yolo26n.pt epochs=30 imgsz=640`.

## Log a run (ADT-9)

```bash
python -m auto_detection_tracking --log logs/run1.csv
```

Writes one CSV row per frame: capture time (`stamp_ns`), capture-to-command latency (`latency_ms`), the selected class and tracking `state`, the target's label, score and box, the angle offset (`yaw_deg`, `pitch_deg`, `off_centre_deg`), and every detection as JSON. It's for measuring DI-7, DI-8 and DI-9 in ADT-10. Only metadata is logged, never images (DI-22). `logs/` is gitignored. The top-right of the window shows FPS and the same latency.

## Score a model on clips (ADT-5)

`scripts/score_clips.py` measures a model on video clips without labelling any frames. For each clip you list the classes that are in it for the whole clip, so anything else the model reports is a mistake. Frames stay in memory; nothing is saved (DI-22).

```bash
cd src/auto_detection_tracking
PYTHONPATH=. python scripts/score_clips.py --model path/to/best.pt --imgsz 1280 \
    --classes syringe adson_forceps needle_driver \
    --clip path/to/forceps_1.mov adson_forceps \
    --clip path/to/all_1.mov syringe,adson_forceps,needle_driver
```

`--imgsz` is the model input size (default 640). Each class is tracked as if the operator selected it on the first frame, with the same `acquire_score`, `min_score` and `lost_after_s` as the app. Times are clip time, so they don't depend on how fast the computer is.

Per clip, one row per class:

| Column | Class in the clip | Class not in the clip |
|---|---|---|
| detected % | Frames where the model boxed it: the detection rate | Frames with a box for it anyway: a confusion (DI-8) |
| tracked % | Frames where the tracker was locked on it | Should be 0; any lock is on the wrong object (DI-8) |
| first lock (s) | Seconds until the tracker first locked on (DI-7 allows 3 s) | Should be "never" |

Then a summary that adds up the clips with the same contents (for example all forceps-only takes):

| Column | Meaning |
|---|---|
| takes | Number of clips in the group |
| detected %, tracked % | As above, over all the group's frames |
| locked in <=3 s | Clips where the tracker locked on within 3 s (DI-7). Want all of them for a class in the clips |
| ever locked | Clips where it locked on at all. Want 0 for a class not in the clips (DI-8) |

Without labels it can't tell which object a box is on: with several instruments in view, a "forceps" box on the needle driver still counts as detected. Labelled frames (`yolo detect val`) catch that.

## Test

```bash
cd src/auto_detection_tracking
python -m pytest test
```
