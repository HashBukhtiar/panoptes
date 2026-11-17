"""Target selection (ADT-6) and tracking between frames (ADT-7)."""
import math

SEARCHING, TRACKING, HOLDING, LOST = "searching", "tracking", "holding", "lost"


def pick(detections, label):
    """Return the highest-scoring detection with this label, or None.

    Only one of each object is on the table, so a second box with the same label is a
    misdetection. Other labels are never returned, whatever their score (DI-8).
    """
    matches = [d for d in detections if d[0] == label]
    return max(matches, key=lambda d: d[1], default=None)


def centre(box):
    x1, y1, x2, y2 = box
    return (x1 + x2) / 2, (y1 + y2) / 2


class Tracker:
    """Follows the selected target from frame to frame.

    States: searching (not seen since it was selected), tracking (detected this frame),
    holding (missed for less than lost_after_s; the last box is kept), lost (missed for
    longer; the last box is kept as the last known position).
    """

    def __init__(self, lost_after_s):
        self.lost_after_ns = int(lost_after_s * 1e9)
        self.select(None)

    def select(self, label):
        """Target a new label (None clears it); tracking starts over."""
        self.label, self.last, self.last_seen_ns = label, None, None

    def update(self, detections, stamp_ns):
        """Return (state, detection) for the frame captured at stamp_ns.

        state is None when nothing is selected. detection is this frame's box when
        tracking, the last one seen when holding or lost, and None when searching.
        """
        if self.label is None:
            return None, None
        matches = [d for d in detections if d[0] == self.label]
        locked = self.last is not None and stamp_ns - self.last_seen_ns < self.lost_after_ns
        if matches:
            if locked:  # follow the box nearest the last position, not the highest score
                last_centre = centre(self.last[2])
                self.last = min(matches, key=lambda d: math.dist(centre(d[2]), last_centre))
            else:  # acquire, or reacquire after being lost
                self.last = pick(matches, self.label)
            self.last_seen_ns = stamp_ns
            return TRACKING, self.last
        # ponytail: holds the last box rather than predicting motion; add constant-velocity
        # prediction if fast movement makes DI-9 fail.
        if self.last is None:
            return SEARCHING, None
        return (HOLDING if locked else LOST), self.last
