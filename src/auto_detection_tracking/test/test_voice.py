"""AI-1: each target has one spoken command, and only an exact command counts."""
from auto_detection_tracking.voice import commands

TARGETS = {"fork": "fork", "forceps": "adson_forceps", "needle driver": "needle_driver"}


def test_each_target_has_one_track_phrase_and_stop_clears():
    assert commands(TARGETS) == {"track fork": "fork", "track forceps": "adson_forceps",
                                 "track needle driver": "needle_driver", "stop tracking": None}


def test_a_command_inside_other_speech_is_not_a_command():
    # What the recognizer outputs for "now we'll track the forceps" and other near misses.
    phrases = commands(TARGETS)
    for heard in ["[unk] track forceps [unk]", "track forceps [unk]", "forceps", "track", "[unk]", ""]:
        assert heard not in phrases
