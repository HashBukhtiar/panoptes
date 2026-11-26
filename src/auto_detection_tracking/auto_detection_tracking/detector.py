"""Object detector: an Ultralytics YOLO model, limited to the configured classes (ADT-3)."""
import numpy as np
import torch
from ultralytics import YOLO


def class_ids(names, wanted):
    """Map class names to the model's ids, failing loudly on a name the model doesn't know."""
    ids = {name: i for i, name in names.items()}
    missing = [name for name in wanted if name not in ids]
    if missing:
        raise ValueError(f"Model has no class {missing}. Known classes: {sorted(ids)}")
    return [ids[name] for name in wanted]


def owners(groups):
    """{class: group} for [(group, classes), ...], in order (ADT-13). A class in two groups fails loudly,
    since the app couldn't tell which model to run for it."""
    owner = {}
    for group, classes in groups:
        for c in classes:
            if c in owner:
                raise ValueError(f"Class {c!r} is in both {owner[c]!r} and {group!r}")
            owner[c] = group
    return owner


def load(weights, classes, min_score, imgsz=640):
    """Return detect(bgr_image) -> [(label, score, (x1, y1, x2, y2)), ...] in pixels.

    weights: a model file; Ultralytics downloads its own pretrained ones (e.g. yolo26n.pt)
    into the current folder on first use.
    imgsz: the size frames are resized to for the model. Larger finds smaller objects but is slower.
    """
    model = YOLO(weights)
    keep = class_ids(model.names, classes)
    # Ultralytics picks CUDA when present but never picks Apple's GPU on its own.
    device = "mps" if torch.backends.mps.is_available() else None

    def detect(bgr):
        # agnostic_nms: where boxes of different classes overlap, keep only the highest-scoring one,
        # so a spoon can't also be reported as a lower-scoring "knife" (DI-8).
        # ponytail: would also drop one of two real instruments stacked almost exactly on top of
        # each other (overlap above Ultralytics' default 0.7); revisit if ADT-10 shows that.
        r = model(bgr, classes=keep, conf=min_score, agnostic_nms=True, imgsz=imgsz, device=device, verbose=False)[0]
        return [(r.names[int(c)], float(s), tuple(int(v) for v in box))
                for c, s, box in zip(r.boxes.cls, r.boxes.conf, r.boxes.xyxy)]

    # The first call takes ~1 s of one-off setup. Pay it here, not on a live frame, where it would
    # freeze the video and eat into DI-7's 3 s. ponytail: warmed for 1280x720 frames (the webcam);
    # another frame shape pays a smaller first-call cost.
    detect(np.zeros((720, 1280, 3), np.uint8))
    return detect
