from agt_pointcloud_tools.static_detector import (
    ImuSample,
    StaticDetectorConfig,
    detect_static_segments,
)


def _sample(t, gyro=0.005, acc=9.81):
    return ImuSample(t, gyro, 0.0, 0.0, 0.0, 0.0, acc)


def test_detects_long_static_interval():
    samples = [_sample(i * 0.05) for i in range(120)]
    segments = detect_static_segments(
        samples,
        StaticDetectorConfig(window_sec=0.5, min_duration_sec=2.0),
    )
    assert segments
    assert segments[0].end - segments[0].start >= 2.0


def test_rejects_rotating_interval():
    samples = [_sample(i * 0.05, gyro=0.5) for i in range(120)]
    segments = detect_static_segments(
        samples,
        StaticDetectorConfig(window_sec=0.5, min_duration_sec=2.0),
    )
    assert segments == []
