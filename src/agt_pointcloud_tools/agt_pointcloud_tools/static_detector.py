from dataclasses import dataclass
from math import sqrt
from statistics import fmean, pstdev
from typing import Iterable, List, Sequence


@dataclass(frozen=True)
class ImuSample:
    t: float
    gx: float
    gy: float
    gz: float
    ax: float
    ay: float
    az: float


@dataclass(frozen=True)
class StaticSegment:
    start: float
    end: float
    score: float


@dataclass(frozen=True)
class StaticDetectorConfig:
    window_sec: float = 0.75
    min_duration_sec: float = 2.0
    max_gyro_mean: float = 0.06
    max_gyro_std: float = 0.025
    max_acc_norm_std: float = 0.15
    merge_gap_sec: float = 0.35


def _norm3(x: float, y: float, z: float) -> float:
    return sqrt(x * x + y * y + z * z)


def detect_static_segments(
    samples: Sequence[ImuSample],
    cfg: StaticDetectorConfig = StaticDetectorConfig(),
) -> List[StaticSegment]:
    if len(samples) < 2:
        return []

    windows = []
    left = 0
    for right in range(len(samples)):
        while samples[right].t - samples[left].t > cfg.window_sec and left < right:
            left += 1
        window = samples[left:right + 1]
        if len(window) < 4:
            continue

        gyro = [_norm3(s.gx, s.gy, s.gz) for s in window]
        accel = [_norm3(s.ax, s.ay, s.az) for s in window]
        gyro_mean = fmean(gyro)
        gyro_std = pstdev(gyro)
        acc_std = pstdev(accel)

        is_static = (
            gyro_mean <= cfg.max_gyro_mean
            and gyro_std <= cfg.max_gyro_std
            and acc_std <= cfg.max_acc_norm_std
        )
        if is_static:
            quality = max(
                0.0,
                1.0 - max(
                    gyro_mean / max(cfg.max_gyro_mean, 1e-9),
                    gyro_std / max(cfg.max_gyro_std, 1e-9),
                    acc_std / max(cfg.max_acc_norm_std, 1e-9),
                ),
            )
            windows.append((window[0].t, window[-1].t, quality))

    if not windows:
        return []

    merged = []
    start, end, scores = windows[0][0], windows[0][1], [windows[0][2]]
    for w_start, w_end, score in windows[1:]:
        if w_start - end <= cfg.merge_gap_sec:
            end = max(end, w_end)
            scores.append(score)
        else:
            if end - start >= cfg.min_duration_sec:
                merged.append(StaticSegment(start, end, fmean(scores)))
            start, end, scores = w_start, w_end, [score]

    if end - start >= cfg.min_duration_sec:
        merged.append(StaticSegment(start, end, fmean(scores)))

    return sorted(merged, key=lambda s: (-(s.end - s.start), -s.score))
