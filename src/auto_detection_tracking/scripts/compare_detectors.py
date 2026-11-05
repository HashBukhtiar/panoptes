"""ADT-3: compare pretrained COCO detectors on the same recorded clips of the stand-in objects.

Every object in ON_DESK must be visible for the whole clip, one of each.
Frames are processed in memory only; nothing is saved (DI-22).

Run from src/auto_detection_tracking/:
    python scripts/compare_detectors.py CLIP [CLIP ...]
"""
import sys
import time
from collections import Counter

import cv2
import numpy as np
import torch

TARGETS = ("toothbrush", "fork", "scissors")  # stand-ins for syringe, Adson forceps, needle driver
ON_DESK = TARGETS + ("knife", "spoon")  # knife and spoon are distractors
THRESHOLDS = (0.25, 0.5)
DEVICE = "mps" if torch.backends.mps.is_available() else "cpu"


# Each candidate returns detect(bgr_image) -> [(label, score), ...] for scores >= 0.1.

def ultralytics_yolo():
    from ultralytics import YOLO
    model = YOLO(YOLO_WEIGHTS)

    def detect(bgr):
        r = model(bgr, device=DEVICE, conf=0.1, verbose=False)[0]
        return [(r.names[int(c)], float(s)) for c, s in zip(r.boxes.cls, r.boxes.conf)]
    return detect


def rt_detr():
    from transformers import AutoImageProcessor, AutoModelForObjectDetection
    processor = AutoImageProcessor.from_pretrained(RT_DETR_MODEL)
    model = AutoModelForObjectDetection.from_pretrained(RT_DETR_MODEL).to(DEVICE).eval()

    def detect(bgr):
        inputs = processor(images=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), return_tensors="pt").to(DEVICE)
        with torch.no_grad():
            out = model(**inputs)
        r = processor.post_process_object_detection(out, target_sizes=[bgr.shape[:2]], threshold=0.1)[0]
        return [(model.config.id2label[int(l)], float(s)) for l, s in zip(r["labels"], r["scores"])]
    return detect


def torchvision_faster_rcnn():
    from torchvision.models.detection import (FasterRCNN_MobileNet_V3_Large_FPN_Weights,
                                              fasterrcnn_mobilenet_v3_large_fpn)
    weights = FasterRCNN_MobileNet_V3_Large_FPN_Weights.DEFAULT
    model = fasterrcnn_mobilenet_v3_large_fpn(weights=weights, box_score_thresh=0.1).to(DEVICE).eval()
    names = weights.meta["categories"]

    def detect(bgr):
        x = torch.from_numpy(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)).permute(2, 0, 1).float().div(255)
        with torch.no_grad():
            r = model([x.to(DEVICE)])[0]
        return [(names[int(l)], float(s)) for l, s in zip(r["labels"], r["scores"])]
    return detect


YOLO_WEIGHTS = "yolo26n.pt"  # Ultralytics' current smallest model; downloads into the folder you run from
RT_DETR_MODEL = "PekingU/rtdetr_v2_r18vd"
CANDIDATES = {
    f"Ultralytics {YOLO_WEIGHTS}": ultralytics_yolo,
    f"RT-DETR {RT_DETR_MODEL}": rt_detr,
    "torchvision Faster R-CNN MobileNetV3": torchvision_faster_rcnn,
}


def run(detect, clips):
    detections, ms = [], []
    for clip in clips:
        cap = cv2.VideoCapture(clip)
        while True:
            ok, bgr = cap.read()
            if not ok:
                break
            start = time.perf_counter()
            detections.append(detect(bgr))
            ms.append((time.perf_counter() - start) * 1000)
        cap.release()
    return detections, ms[5:]  # first frames include warm-up


def report(name, detections, ms):
    print(f"\n### {name}  ({len(detections)} frames, {DEVICE})")
    print(f"Time per frame: mean {np.mean(ms):.0f} ms, p95 {np.percentile(ms, 95):.0f} ms")
    for th in THRESHOLDS:
        counts = [Counter(label for label, s in frame if s >= th) for frame in detections]
        n = len(counts)
        print(f"\nScore >= {th}\n\n| object | found in % of frames | 2+ boxes in % of frames |\n|---|---|---|")
        for label in ON_DESK:
            found = sum(c[label] >= 1 for c in counts) / n * 100
            doubled = sum(c[label] >= 2 for c in counts) / n * 100
            print(f"| {label}{' (distractor)' if label not in TARGETS else ''} | {found:.0f} | {doubled:.0f} |")
        others = Counter(label for c in counts for label in c if label not in ON_DESK)
        print("\nOther labels (% of frames): "
              + (", ".join(f"{label} {k / n * 100:.0f}" for label, k in others.most_common(6)) or "none"))


if __name__ == "__main__":
    clips = sys.argv[1:]
    for name, make in CANDIDATES.items():
        report(name, *run(make(), clips))
