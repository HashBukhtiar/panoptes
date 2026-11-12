"""Object detector: an Ultralytics YOLO model, limited to the configured classes (ADT-3)."""
import torch
from ultralytics import YOLO


def class_ids(names, wanted):
    """Map class names to the model's ids, failing loudly on a name the model doesn't know."""
    ids = {name: i for i, name in names.items()}
    missing = [name for name in wanted if name not in ids]
    if missing:
        raise ValueError(f"Model has no class {missing}. Known classes: {sorted(ids)}")
    return [ids[name] for name in wanted]


def load(weights, classes, min_score):
    """Return detect(bgr_image) -> [(label, score, (x1, y1, x2, y2)), ...] in pixels.

    weights: a model file; Ultralytics downloads its own pretrained ones (e.g. yolo26n.pt)
    into the current folder on first use.
    """
    model = YOLO(weights)
    keep = class_ids(model.names, classes)
    # Ultralytics picks CUDA when present but never picks Apple's GPU on its own.
    device = "mps" if torch.backends.mps.is_available() else None

    def detect(bgr):
        r = model(bgr, classes=keep, conf=min_score, device=device, verbose=False)[0]
        return [(r.names[int(c)], float(s), tuple(int(v) for v in box))
                for c, s, box in zip(r.boxes.cls, r.boxes.conf, r.boxes.xyxy)]
    return detect
