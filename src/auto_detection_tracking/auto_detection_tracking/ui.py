"""Operator view for autonomous mode (ADT-14): the camera image, the targets, and where to turn.

Lines, rings and boxes are drawn with OpenCV. Panels and text use Pillow for real fonts and
translucency, but only on the panel areas: compositing the whole frame costs about 7 ms per frame.
ponytail: sizes are in pixels for 1280x720 frames; scale them with the image width for the ZED at 1080p.
"""
import math
from functools import lru_cache

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from auto_detection_tracking.pointing import off_centre, readout
from auto_detection_tracking.target import HOLDING, LOST, SEARCHING, TRACKING, centre

FONTS = ["/System/Library/Fonts/SFNS.ttf",  # macOS system font
         "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"]  # common on Linux, e.g. the Jetson
# Colours are RGB; OpenCV calls take bgr(colour).
GREEN, AMBER, RED, GREY, WHITE, BLUE = (46, 204, 113), (241, 196, 15), (231, 76, 60), (150, 150, 150), (255, 255, 255), (52, 152, 219)
INK = (12, 20, 12)  # dark text on bright pills
PANEL = (14, 17, 22, 165)  # translucent dark, so what's underneath still shows
MUTED_TEXT = (150, 160, 170)
STATE_COLOUR = {SEARCHING: GREY, TRACKING: GREEN, HOLDING: AMBER, LOST: RED}
MARGIN = 16
LIST_W, ROW = 220, 28  # target list: narrow, so targets near the left edge stay visible


def bgr(rgb):
    return rgb[::-1]


def pretty(label):
    """adson_forceps -> Adson forceps."""
    return label.replace("_", " ").capitalize()


def ring_px(deg, f):
    """Radius in pixels of the ring deg degrees off centre, for a focal length of f pixels."""
    return round(f * math.tan(math.radians(deg)))


@lru_cache(maxsize=None)
def font(size, weight="Regular"):
    """The first available font in FONTS (weight applies to variable fonts), else Pillow's own."""
    for path in FONTS:
        try:
            f = ImageFont.truetype(path, size)
        except OSError:
            continue
        try:
            f.set_variation_by_name(weight)
        except (OSError, ValueError):  # not a variable font: one weight only
            pass
        return f
    return ImageFont.load_default(size)


def layer(image, x, y, w, h, draw):
    """Draw with Pillow onto one region of image, in place. draw(d) uses coordinates relative to (x, y)."""
    height, width = image.shape[:2]
    x1, y1, x2, y2 = max(x, 0), max(y, 0), min(x + w, width), min(y + h, height)
    if x1 >= x2 or y1 >= y2:
        return
    over = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    draw(ImageDraw.Draw(over))
    over = over.crop((x1 - x, y1 - y, x2 - x, y2 - y))
    region = Image.fromarray(np.ascontiguousarray(image[y1:y2, x1:x2, ::-1])).convert("RGBA")
    image[y1:y2, x1:x2] = np.asarray(Image.alpha_composite(region, over).convert("RGB"))[:, :, ::-1]


def pill(d, x, y, text, size, fill, ink, weight="Semibold"):
    """A rounded label with its left edge at x and its middle at y; returns its width."""
    f = font(size, weight)
    w = round(d.textlength(text, font=f)) + size
    d.rounded_rectangle((x, y - size, x + w, y + size), size, fill=fill)
    d.text((x + w / 2, y), text, font=f, fill=ink, anchor="mm")
    return w


def tag(image, x, y, text, fill, ink, size=14):
    """A pill label whose bottom-left corner is at (x, y)."""
    w = round(font(size, "Semibold").getlength(text)) + size
    layer(image, x, y - 2 * size, w, 2 * size, lambda d: pill(d, 0, size, text, size, fill, ink))


def brackets(image, box, colour, thickness=3):
    """Corner brackets around box instead of a full rectangle."""
    x1, y1, x2, y2 = box
    n = max(6, min(18, (x2 - x1) // 3, (y2 - y1) // 3))
    for x, y, sx, sy in ((x1, y1, 1, 1), (x2, y1, -1, 1), (x1, y2, 1, -1), (x2, y2, -1, -1)):
        cv2.line(image, (x, y), (x + sx * n, y), bgr(colour), thickness, cv2.LINE_AA)
        cv2.line(image, (x, y), (x, y + sy * n), bgr(colour), thickness, cv2.LINE_AA)


def render(image, *, groups, selected, state, target, detections, angles, f, centred_deg, fps, latency_ms, mic=None):
    """Draw the operator view onto image, in place.

    groups: [(group name, [class, ...]), ...] in key order. target: (label, score, box) or None.
    angles: the target's (yaw, pitch) or None. f: focal length in pixels. mic: None when voice is off,
    else {"muted": bool, "warning": str or None, "heard": the phrase just heard or None}.
    """
    height, width = image.shape[:2]
    mid = (width // 2, height // 2)
    colour = STATE_COLOUR.get(state, GREY)
    centred = bool(state == TRACKING and angles and math.degrees(off_centre(*angles)) <= centred_deg)

    # Other detections: thin grey boxes, ignored (DI-8)
    for d in detections:
        if d is not target:
            label, score, box = d
            cv2.rectangle(image, box[:2], box[2:], bgr(GREY), 1, cv2.LINE_AA)
            tag(image, box[0], box[1] - 3, f"{pretty(label)} {score:.2f}", (40, 44, 52, 170), (210, 210, 210), 11)

    # Centre: crosshair and the centred ring (DI-7), green once the target is inside it
    ring = GREEN if centred else WHITE
    cv2.circle(image, mid, ring_px(centred_deg, f), bgr(ring), 2 if centred else 1, cv2.LINE_AA)
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        cv2.line(image, (mid[0] + 8 * dx, mid[1] + 8 * dy), (mid[0] + 22 * dx, mid[1] + 22 * dy), bgr(ring), 2,
                 cv2.LINE_AA)

    # Top left: status card
    def status(d):
        d.rounded_rectangle((0, 0, 314, 88), 14, fill=PANEL)
        d.text((16, 12), "PANOPTES  ·  AUTONOMOUS", font=font(13, "Semibold"), fill=MUTED_TEXT)
        if selected:
            w = pill(d, 16, 54, (state or SEARCHING).upper(), 15, colour + (255,), WHITE if state == LOST else INK, "Bold")
            d.text((30 + w, 54), pretty(selected), font=font(22, "Semibold"), fill=WHITE, anchor="lm")
        else:
            pill(d, 16, 54, "NO TARGET", 15, (70, 74, 82, 255), WHITE, "Bold")
    layer(image, MARGIN, MARGIN, 314, 88, status)

    # Left: every target with its key, the selected one highlighted
    rows = sum(1 + len(classes) for _name, classes in groups)
    list_h = 16 + ROW * rows

    def targets(d):
        d.rounded_rectangle((0, 0, LIST_W, list_h), 14, fill=PANEL)
        y, key = 10, 1
        for name, classes in groups:
            d.text((16, y + 4), name.replace("_", "-").upper(), font=font(12, "Semibold"), fill=(130, 140, 150))
            y += ROW
            for label in classes:
                on = label == selected
                if on:
                    d.rounded_rectangle((8, y - 3, LIST_W - 8, y + 23), 8, fill=BLUE + (110,))
                d.rounded_rectangle((16, y, 36, y + 20), 5, outline=(220, 220, 220) if on else (110, 110, 110))
                d.text((26, y + 10), str(key), font=font(13, "Semibold"), fill=WHITE if on else GREY, anchor="mm")
                d.text((46, y + 10), pretty(label), font=font(16, "Semibold" if on else "Regular"),
                       fill=WHITE if on else (185, 185, 185), anchor="lm")
                y += ROW
                key += 1
    layer(image, MARGIN, MARGIN + 100, LIST_W, list_h, targets)

    # Top right: camera-active indicator (DI-22) and performance
    def camera(d):
        d.rounded_rectangle((0, 0, 180, 34), 17, fill=PANEL)
        d.ellipse((14, 11, 26, 23), fill=RED)
        d.text((36, 17), "CAMERA ON", font=font(15, "Semibold"), fill=WHITE, anchor="lm")
        d.text((90, 54), f"{fps:.0f} fps  ·  {latency_ms:.0f} ms", font=font(13), fill=(200, 200, 200), anchor="mm")
    layer(image, width - MARGIN - 180, MARGIN, 180, 66, camera)

    # After the side panels, so they never hide it: the target, and an arrow showing which way to turn
    if target and state in (TRACKING, HOLDING, LOST):
        label, score, box = target
        tc = tuple(int(v) for v in centre(box))
        length = math.dist(mid, tc)
        if not centred and length > 30:
            cv2.arrowedLine(image, mid, tc, bgr(colour), 2, cv2.LINE_AA, tipLength=min(0.3, 14 / length))
        if state == LOST:  # the box is only the last known position
            cv2.drawMarker(image, tc, bgr(RED), cv2.MARKER_TILTED_CROSS, 26, 3, cv2.LINE_AA)
            tag(image, tc[0] - 40, tc[1] - 18, f"{pretty(label)}  last seen", RED + (235,), WHITE)
        else:
            brackets(image, box, colour)
            text = f"{pretty(label)}  {score:.2f}" if state == TRACKING else f"{pretty(label)}  held"
            y = box[1] - 6 if box[1] > 40 else box[3] + 34  # below the box if there's no room above
            tag(image, box[0], y, text, colour + (240,), INK)

    # Bottom bar last, so its text stays readable over a large target
    n = sum(len(classes) for _name, classes in groups)
    if not selected:
        message, message_colour = f"Say “track …” or press 1–{n}", MUTED_TEXT
    elif state in (None, SEARCHING):
        message, message_colour = f"Searching for {pretty(selected)}…", MUTED_TEXT
    else:
        message, message_colour = readout(state, *angles, centred_deg), GREEN if centred else colour
    hints = f"1–{n} target   0 clear   {'M mute   ' if mic else ''}Q quit"

    def bar(d):
        w = width - 2 * MARGIN
        d.rounded_rectangle((0, 0, w, 56), 14, fill=PANEL)
        if mic:
            dot = RED if mic["warning"] else GREY if mic["muted"] else GREEN
            d.ellipse((20, 14, 32, 26), fill=dot)
            d.text((42, 20), mic["warning"] or ("MIC MUTED" if mic["muted"] else "MIC LISTENING"),
                   font=font(15, "Semibold"), fill=WHITE, anchor="lm")
            if mic["heard"]:
                d.text((42, 40), f"heard “{mic['heard']}”", font=font(13), fill=(170, 180, 190), anchor="lm")
        d.text((w / 2, 28), message, font=font(26 if state in (TRACKING, HOLDING, LOST) else 20, "Bold"),
               fill=message_colour, anchor="mm")
        d.text((w - 20, 28), hints, font=font(13), fill=MUTED_TEXT, anchor="rm")
    layer(image, MARGIN, height - MARGIN - 56, width - 2 * MARGIN, 56, bar)
