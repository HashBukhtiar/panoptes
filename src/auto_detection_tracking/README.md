# auto_detection_tracking

Autonomous-mode vision: find the operator's selected instrument and keep it centred. So far it opens a camera source, shows it live (ADT-1), and boxes the configured objects with a pretrained YOLO model (ADT-3), lets you pick one as the target (ADT-6), follows it between frames (ADT-7), and shows the direction the camera should turn (ADT-8). It has six targets from two models (ADT-13): stand-ins (fork, spoon, knife) and the real instruments (syringe, Adson forceps, needle driver). See [Models](#models). Targets can be picked by key or by voice (AI-1, see [Voice](#voice-ai-1)).

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

Press `1` to `6` to make a class the target (in the order listed under `detectors`: fork, spoon, knife, syringe, adson_forceps, needle_driver), and `0` to clear it. Only the model that owns the selected target runs, which keeps the video at 30 fps; picking a target from the other group switches models.

The window is the autonomous-mode operator view (ADT-14). Top left, a card shows the selected target and its tracking state; below it, every target with its key, the selected one highlighted. Top right, CAMERA ON with the frame rate and latency. In the centre, a crosshair with a ring `centred_deg` (5°, DI-7) from the centre, which turns green once the target is inside it. The target gets corner brackets and a name tag in its state's colour, plus an arrow from the centre showing which way the camera should turn; other detected objects get thin grey boxes and are ignored (DI-8). The states:

- **searching** (grey): not yet seen with a score of at least `acquire_score` since you selected it. Requiring a confident box to start keeps weak look-alikes from being picked up (DI-8).
- **tracking** (green): detected this frame. Once found, the target follows the box nearest its last position, so a higher-scoring look-alike elsewhere doesn't steal it.
- **holding** (amber): missed for less than `lost_after_s`; the last box is kept.
- **lost** (red X at the last known position): missed for longer. The next detection of the class scoring at least `acquire_score`, anywhere, reacquires it.

The bottom bar's large text stands in for the camera command until there's a mechanism to move (ADT-8): `MOVE LEFT 12° · DOWN 4°`, `CENTRED`, or `LOST · MOVE UP` from the last known position. Directions are from the camera's point of view, so facing your webcam, "LEFT" is your right. The degrees come from `hfov_deg`, the camera's horizontal field of view, set for Hashim's MacBook webcam. For another camera, hold a ruler level at distance D from it, read the width W visible across the frame, and set `hfov_deg` to 2·atan(W / 2D) in degrees. Keep macOS Center Stage off (Control Center > Video Effects): it crops and pans the image, so the angles would be wrong.

Quit with `q`, `Esc`, or by closing the window. The window is the camera-active indicator (DI-22): the camera is open only while the window is showing, and nothing is recorded.

The view uses the macOS system font, or DejaVu Sans on Linux if installed, else Pillow's built-in font. Its layout is sized for 1280x720 frames. DI-9's ring isn't drawn yet: whether "central 15°" means 15° from the centre or 15° across is still open.

On macOS, the first webcam run asks for camera permission for your terminal app and exits with "Could not open camera source" while the prompt is up. Allow it, then run again. If no prompt appears, turn your terminal app on under System Settings > Privacy & Security > Camera and restart the terminal. If index 0 opens the wrong camera (e.g. an iPhone through Continuity Camera), try `"1"`.

## Voice (AI-1)

Say **"track fork"**, **"track spoon"**, **"track knife"**, **"track syringe"**, **"track forceps"** or **"track needle driver"** to select a target, and **"stop tracking"** to clear it. Pause briefly before and after: only speech that is exactly one command counts, so a sentence like "now we'll track the forceps" is ignored. Press `m` to mute or unmute. The bottom bar shows MIC LISTENING or MIC MUTED, and the last command heard.

Recognition runs offline on the laptop with [Vosk](https://alphacephei.com/vosk/) and is limited to these phrases. Audio is handled in memory 0.1 s at a time, never saved and never sent anywhere (DI-22). A command takes effect about 0.8 s after you stop speaking (the recognizer waits for a pause), which counts toward DI-7's 3 s. The words to say are the `spoken` lists in the config.

The speech model isn't in git. Download it once into `src/auto_detection_tracking/` (41 MB, Apache-2.0):

```bash
curl -LO https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip && unzip vosk-model-small-en-us-0.15.zip && rm vosk-model-small-en-us-0.15.zip
```

If the model or a microphone is missing, the app says "Voice off" and the keyboard still works; `voice_model: ""` in the config turns voice off on purpose. The first run asks for microphone permission for your terminal app. If it was refused, macOS supplies silence instead of an error, and the window shows "mic is silent: check microphone permission": turn your terminal app on under System Settings > Privacy & Security > Microphone and restart the terminal.

## Models

| Group | Keys | Weights | Input size | Classes |
|---|---|---|---|---|
| `stand_ins` | 1-3 | `yolo26n.pt`: YOLO26n pretrained on COCO, downloaded on first run | 640 | fork, spoon, knife |
| `instruments` | 4-6 | `instruments-public-v1.pt`: YOLO26n fine-tuned on public data (ADT-5) | 1280 | syringe, adson_forceps, needle_driver |

Both files live in `src/auto_detection_tracking/` and are gitignored (`*.pt`). The instrument weights aren't downloadable: copy `instruments-public-v1.pt` from Teams (Documents > Team_01-DHF > 04. Design > 04. Software) into that folder. Without it the app stops with "No such file or directory: 'instruments-public-v1.pt'".

**How good the instrument model is:** it has seen public photos only, none of ours. On 26 labelled webcam frames of our instruments at 60-90 cm (score threshold 0.25, input 1280) it found 5 of 10 syringes, 10 of 15 forceps and 0 of 14 needle drivers, and once called a syringe forceps. The public photos are close-ups (an instrument fills about 40% of the image, against 10-18% in ours), and their needle holders look different from ours. Training on our own photos waits for the ZED camera.

**Training data (CC BY 4.0):** [surgical_tool_detection_6class](https://universe.roboflow.com/mj-4iyes/surgical_tool_detection_6class) by mj-4iyes (its Forcep and Needle_holder classes) and [syringe](https://universe.roboflow.com/syringe-project/syringe-tovbf) by syringe-project, both on Roboflow Universe. Changes: other classes dropped, polygon outlines converted to boxes, merged with `python -m auto_detection_tracking.dataset`, then trained with `yolo detect train model=yolo26n.pt epochs=30 imgsz=640`.

## Log a run (ADT-9)

```bash
python -m auto_detection_tracking --log logs/run1.csv
```

Writes one CSV row per frame: capture time (`stamp_ns`), capture-to-command latency (`latency_ms`), the selected class and tracking `state`, the target's label, score and box, the angle offset (`yaw_deg`, `pitch_deg`, `off_centre_deg`), and every detection as JSON. The first frame after a selection command also records the `command` (e.g. `key 5` or `voice: track forceps`) and `command_ns`, when it was given: the key press, or the end of speech. The time from `command_ns` to the first `tracking` row is the acquisition time (DI-7). It's for measuring DI-7, DI-8 and DI-9 in ADT-10. Only metadata is logged, never images (DI-22). `logs/` is gitignored. The top right of the window shows the frame rate and the same latency.

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
