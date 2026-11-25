"""Merge YOLO-format datasets into one with our class names (ADT-4/5).

Roboflow exports often hold polygon outlines even for detection projects; every label is
converted to a plain box. Classes not in a source's map are dropped from the labels, so
those objects become background. Images are symlinked, not copied.

Run from src/auto_detection_tracking/:
    python -m auto_detection_tracking.dataset OUT_DIR --names syringe adson_forceps needle_driver \
        --source mj_ PATH/surgical_tool_detection_6class "Forcep=adson_forceps,Needle_holder=needle_driver" \
        --source syr_ PATH/syringe "syringe=syringe"
"""
import argparse
import os
from pathlib import Path

import yaml

SPLITS = {"train": "train", "valid": "val", "test": "val"}  # their split -> ours (their test is extra validation)


def to_box(values):
    """(cx, cy, w, h) from a YOLO box (4 numbers) or polygon (x1 y1 x2 y2 ...), normalised 0-1."""
    if len(values) == 4:
        return tuple(values)
    xs, ys = values[0::2], values[1::2]
    x1, x2 = max(min(xs), 0.0), min(max(xs), 1.0)
    y1, y2 = max(min(ys), 0.0), min(max(ys), 1.0)
    return (x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1


def remap(label_text, id_map):
    """Rewrite one label file: keep only mapped class ids, as boxes."""
    lines = []
    for line in label_text.splitlines():
        parts = line.split()
        if not parts or int(parts[0]) not in id_map:
            continue
        cx, cy, w, h = to_box([float(v) for v in parts[1:]])
        if w > 0 and h > 0:
            lines.append(f"{id_map[int(parts[0])]} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
    return "\n".join(lines) + ("\n" if lines else "")


def merge(out, names, sources):
    """sources: [(prefix, dataset_dir, {their_name: our_name})]. Writes out/data.yaml."""
    out = Path(out)
    for prefix, src, name_map in sources:
        src = Path(src)
        theirs = yaml.safe_load((src / "data.yaml").read_text())["names"]
        id_map = {theirs.index(a): names.index(b) for a, b in name_map.items()}
        for their_split, our_split in SPLITS.items():
            for image in sorted((src / their_split / "images").glob("*")) if (src / their_split).is_dir() else []:
                label = src / their_split / "labels" / (image.stem + ".txt")
                stem = prefix + image.stem
                (out / "images" / our_split).mkdir(parents=True, exist_ok=True)
                (out / "labels" / our_split).mkdir(parents=True, exist_ok=True)
                link = out / "images" / our_split / (stem + image.suffix)
                if not link.exists():
                    os.symlink(image.resolve(), link)
                text = label.read_text() if label.exists() else ""
                (out / "labels" / our_split / (stem + ".txt")).write_text(remap(text, id_map))
    (out / "data.yaml").write_text(yaml.safe_dump(
        {"path": str(out.resolve()), "train": "images/train", "val": "images/val", "names": dict(enumerate(names))}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Merge YOLO datasets into one with our class names.")
    parser.add_argument("out")
    parser.add_argument("--names", nargs="+", required=True)
    parser.add_argument("--source", nargs=3, action="append", required=True, metavar=("PREFIX", "DIR", "MAP"),
                        help='e.g. mj_ path/to/dataset "Forcep=adson_forceps,Needle_holder=needle_driver"')
    args = parser.parse_args()
    merge(args.out, args.names,
          [(p, d, dict(pair.split("=") for pair in m.split(","))) for p, d, m in args.source])
    for split in ("train", "val"):
        labels = list((Path(args.out) / "labels" / split).glob("*.txt"))
        counts = [0] * len(args.names)
        for f in labels:
            for line in f.read_text().splitlines():
                counts[int(line.split()[0])] += 1
        print(split, len(labels), "images,", dict(zip(args.names, counts)), "boxes")
