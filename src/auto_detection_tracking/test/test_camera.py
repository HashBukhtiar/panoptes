"""ADT-1: the camera source yields every frame, timestamped and paced like a live camera."""
import cv2
import numpy as np
import pytest

from auto_detection_tracking.camera import frames

FPS = 20
N_FRAMES = 10


def test_video_file_yields_every_frame_paced_to_its_fps(tmp_path):
    # Synthetic flat-colour frames, not camera images, so nothing here can show a person (DI-22).
    path = str(tmp_path / "synthetic.avi")
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"), FPS, (64, 48))
    for i in range(N_FRAMES):
        writer.write(np.full((48, 64, 3), i * 20, np.uint8))
    writer.release()

    out = list(frames(path))

    assert len(out) == N_FRAMES
    assert out[0][1].shape == (48, 64, 3)
    gaps_s = np.diff([stamp for stamp, _ in out]) / 1e9
    assert (gaps_s > 0).all()
    assert abs(gaps_s.mean() - 1 / FPS) < 0.01


def test_missing_file_raises():
    with pytest.raises(RuntimeError):
        next(frames("does-not-exist.avi"))
