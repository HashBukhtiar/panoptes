# auto_detection_tracking

Autonomous-mode vision: find the operator's selected instrument and keep it centred. So far it opens a camera source, shows it live (ADT-1), and boxes the configured objects with a pretrained YOLO model (ADT-3). Until the real instruments arrive, the configured objects are stand-ins: fork, spoon and knife.

## Setup (once, from the repo root)

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r src/auto_detection_tracking/requirements.txt
```

## Run

```bash
cd src/auto_detection_tracking
python -m auto_detection_tracking
```

Pick the source in `config/auto_detection_tracking.yaml`: a webcam index (`"0"`) or a path to a video file. To try a different source without editing the file, pass `--source`, e.g. `python -m auto_detection_tracking --source path/to/clip.mov`. Relative paths are resolved from the folder you run in. Video files play at their own frame rate. The same file sets the model, which labels to box (`classes`), and the minimum score. The first run downloads the model weights into the folder you run from; `*.pt` files are gitignored. Quit with `q`, `Esc`, or by closing the window.

The window is the camera-active indicator (DI-22): the camera is open only while the window is showing, and nothing is recorded.

On macOS, the first webcam run asks for camera permission for your terminal app and exits with "Could not open camera source" while the prompt is up. Allow it, then run again. If no prompt appears, turn your terminal app on under System Settings > Privacy & Security > Camera and restart the terminal. If index 0 opens the wrong camera (e.g. an iPhone through Continuity Camera), try `"1"`.

## Test

```bash
cd src/auto_detection_tracking
python -m pytest test
```
