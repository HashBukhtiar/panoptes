"""ADT-6 / DI-8: only the selected class is ever reported as the target."""
from auto_detection_tracking.target import pick

FORK = ("fork", 0.6, (0, 0, 10, 10))
KNIFE_HIGH = ("knife", 0.9, (20, 0, 30, 10))
KNIFE_LOW = ("knife", 0.4, (40, 0, 50, 10))


def test_only_the_selected_label_is_reported_even_if_another_scores_higher():
    assert pick([FORK, KNIFE_HIGH], "fork") == FORK


def test_highest_score_wins_among_duplicates():
    assert pick([KNIFE_LOW, FORK, KNIFE_HIGH], "knife") == KNIFE_HIGH


def test_no_target_when_nothing_is_selected_or_it_is_not_in_view():
    assert pick([FORK, KNIFE_HIGH], None) is None
    assert pick([FORK], "spoon") is None
