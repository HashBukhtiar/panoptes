"""ADT-6 / DI-8: only the selected class is ever reported as the target.
ADT-7: the target is followed between frames, held through short gaps, then reported lost."""
from auto_detection_tracking.target import HOLDING, LOST, SEARCHING, TRACKING, Tracker, pick

FORK = ("fork", 0.6, (0, 0, 10, 10))
KNIFE_HIGH = ("knife", 0.9, (20, 0, 30, 10))
KNIFE_LOW = ("knife", 0.4, (40, 0, 50, 10))
MS = 1_000_000  # nanoseconds per millisecond


def tracker_on(label):
    tracker = Tracker(lost_after_s=0.5)
    tracker.select(label)
    return tracker


def test_only_the_selected_label_is_reported_even_if_another_scores_higher():
    assert pick([FORK, KNIFE_HIGH], "fork") == FORK


def test_highest_score_wins_among_duplicates():
    assert pick([KNIFE_LOW, FORK, KNIFE_HIGH], "knife") == KNIFE_HIGH


def test_no_target_when_nothing_is_selected_or_it_is_not_in_view():
    assert pick([FORK, KNIFE_HIGH], None) is None
    assert pick([FORK], "spoon") is None


def test_acquires_highest_score_then_follows_nearest_even_if_a_duplicate_scores_higher():
    tracker = tracker_on("knife")
    assert tracker.update([KNIFE_LOW, KNIFE_HIGH], 0) == (TRACKING, KNIFE_HIGH)
    moved = ("knife", 0.5, (22, 0, 32, 10))       # the real knife, moved slightly
    look_alike = ("knife", 0.95, (80, 0, 90, 10))  # something else labelled knife
    assert tracker.update([look_alike, moved], 33 * MS) == (TRACKING, moved)


def test_holds_the_last_box_through_a_short_gap_then_reports_lost():
    tracker = tracker_on("fork")
    tracker.update([FORK], 0)
    assert tracker.update([], 400 * MS) == (HOLDING, FORK)
    assert tracker.update([], 600 * MS) == (LOST, FORK)


def test_reacquires_by_highest_score_anywhere_after_being_lost():
    tracker = tracker_on("fork")
    tracker.update([FORK], 0)
    tracker.update([], 600 * MS)
    near_low = ("fork", 0.3, (2, 0, 12, 10))
    far_high = ("fork", 0.8, (100, 100, 110, 110))
    assert tracker.update([near_low, far_high], 700 * MS) == (TRACKING, far_high)


def test_new_selection_starts_over():
    tracker = tracker_on("fork")
    tracker.update([FORK], 0)
    tracker.select("spoon")
    assert tracker.update([FORK], 33 * MS) == (SEARCHING, None)
    tracker.select(None)
    assert tracker.update([FORK], 66 * MS) == (None, None)
