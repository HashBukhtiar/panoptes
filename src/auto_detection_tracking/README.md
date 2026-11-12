# auto_detection_tracking

Autonomous-mode vision: find the operator's selected instrument and keep it centred. So far it opens a camera source, shows it live (ADT-1), and boxes the configured objects with a pretrained YOLO model (ADT-3), lets you pick one as the target (ADT-6), and follows it between frames (ADT-7). Until the real instruments arrive, the configured objects are stand-ins: fork, spoon and knife.

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

Pick the source in `config/auto_detection_tracking.yaml`: a webcam index (`"0"`) or a path to a video file. To try a different source without editing the file, pass `--source`, e.g. `python -m auto_detection_tracking --source path/to/clip.mov`. Relative paths are resolved from the folder you run in. Video files play at their own frame rate. The same file sets the model, which labels to box (`classes`), and the minimum score. The first run downloads the model weights into the folder you run from; `*.pt` files are gitignored.

Press `1`, `2` or `3` to make a class the target (in the order listed under `classes`), and `0` to clear it. The target gets a thick green box; other detected objects get thin grey boxes and are ignored (DI-8). The status line shows the tracking state:

- **searching**: not seen since you selected it.
- **tracking** (green box): detected this frame. Once found, the target follows the box nearest its last position, so a higher-scoring look-alike elsewhere doesn't steal it.
- **holding** (yellow box): missed for less than `lost_after_s`; the last box is kept.
- **lost** (red cross at the last known position): missed for longer. The next detection of the class, anywhere, reacquires it.

Quit with `q`, `Esc`, or by closing the window. The window is the camera-active indicator (DI-22): the camera is open only while the window is showing, and nothing is recorded.

On macOS, the first webcam run asks for camera permission for your terminal app and exits with "Could not open camera source" while the prompt is up. Allow it, then run again. If no prompt appears, turn your terminal app on under System Settings > Privacy & Security > Camera and restart the terminal. If index 0 opens the wrong camera (e.g. an iPhone through Continuity Camera), try `"1"`.

## Test

```bash
cd src/auto_detection_tracking
python -m pytest test
```
