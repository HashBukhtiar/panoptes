"""Target selection (ADT-6): report only the operator's chosen class as the target."""


def pick(detections, label):
    """Return the highest-scoring detection with this label, or None.

    Only one of each object is on the table, so a second box with the same label is a
    misdetection. Other labels are never returned, whatever their score (DI-8).
    """
    matches = [d for d in detections if d[0] == label]
    return max(matches, key=lambda d: d[1], default=None)
