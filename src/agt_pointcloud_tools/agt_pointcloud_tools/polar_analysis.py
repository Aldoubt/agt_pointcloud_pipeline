from collections import defaultdict
from dataclasses import dataclass
from math import atan2, cos, degrees, floor, hypot, pi, radians, sin
from statistics import fmean
from typing import Dict, Iterable, List, Mapping, MutableMapping, Sequence, Tuple

from .bag_cloud import CloudFrame


@dataclass(frozen=True)
class RigidTransform:
    translation: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class PolarAnalysisConfig:
    angle_bin_deg: float = 2.0
    range_bin_m: float = 0.10
    min_range_m: float = 0.20
    max_range_m: float = 2.00
    z_min_m: float = -0.20
    z_max_m: float = 1.50
    search_center_deg: float = 180.0
    search_width_deg: float = 120.0
    persistence_min: float = 0.80
    min_persistent_range_bins: int = 2
    angle_padding_deg: float = 2.0
    range_padding_m: float = 0.10
    z_padding_m: float = 0.05


@dataclass
class BinStats:
    frame_hits: int = 0
    point_count: int = 0
    r_min: float = float('inf')
    r_max: float = float('-inf')
    z_min: float = float('inf')
    z_max: float = float('-inf')


@dataclass(frozen=True)
class SectorSuggestion:
    center_deg: float
    width_deg: float
    min_range_m: float
    max_range_m: float
    z_min_m: float
    z_max_m: float
    confidence: float
    supporting_angle_bins: int
    supporting_polar_bins: int


@dataclass
class PolarAnalysisResult:
    frame_count: int
    bin_stats: Dict[Tuple[int, int], BinStats]
    suggestion: SectorSuggestion | None


def _rotation_matrix(rpy_deg):
    roll, pitch, yaw = (radians(v) for v in rpy_deg)
    cr, sr = cos(roll), sin(roll)
    cp, sp = cos(pitch), sin(pitch)
    cy, sy = cos(yaw), sin(yaw)
    # Rz(yaw) * Ry(pitch) * Rx(roll)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def transform_point(point, transform: RigidTransform):
    x, y, z = point
    r = _rotation_matrix(transform.rpy_deg)
    tx, ty, tz = transform.translation
    return (
        r[0][0] * x + r[0][1] * y + r[0][2] * z + tx,
        r[1][0] * x + r[1][1] * y + r[1][2] * z + ty,
        r[2][0] * x + r[2][1] * y + r[2][2] * z + tz,
    )


def _angle360(x, y):
    value = degrees(atan2(y, x))
    return value if value >= 0.0 else value + 360.0


def _angular_delta_deg(value, center):
    return ((value - center + 180.0) % 360.0) - 180.0


def _circular_mean_deg(values, weights):
    sx = sum(w * cos(radians(v)) for v, w in zip(values, weights))
    sy = sum(w * sin(radians(v)) for v, w in zip(values, weights))
    value = degrees(atan2(sy, sx))
    return value if value >= 0.0 else value + 360.0


def _group_circular(indices: Sequence[int], total_bins: int):
    if not indices:
        return []
    sorted_idx = sorted(set(indices))
    groups = [[sorted_idx[0]]]
    for idx in sorted_idx[1:]:
        if idx == groups[-1][-1] + 1:
            groups[-1].append(idx)
        else:
            groups.append([idx])
    if len(groups) > 1 and groups[0][0] == 0 and groups[-1][-1] == total_bins - 1:
        groups[0] = groups[-1] + groups[0]
        groups.pop()
    return groups


def analyze_polar_persistence(
    frames: Iterable[CloudFrame],
    transform: RigidTransform,
    cfg: PolarAnalysisConfig = PolarAnalysisConfig(),
) -> PolarAnalysisResult:
    if cfg.angle_bin_deg <= 0.0 or cfg.range_bin_m <= 0.0:
        raise ValueError('bin sizes must be positive')
    if cfg.max_range_m <= cfg.min_range_m:
        raise ValueError('max_range_m must be greater than min_range_m')

    stats: Dict[Tuple[int, int], BinStats] = defaultdict(BinStats)
    frame_count = 0
    total_angle_bins = max(1, int(round(360.0 / cfg.angle_bin_deg)))

    for frame in frames:
        occupied = set()
        frame_count += 1
        for source_point in frame.points:
            x, y, z = transform_point(source_point, transform)
            r = hypot(x, y)
            if r < cfg.min_range_m or r > cfg.max_range_m:
                continue
            if z < cfg.z_min_m or z > cfg.z_max_m:
                continue

            angle = _angle360(x, y)
            if abs(_angular_delta_deg(angle, cfg.search_center_deg)) > 0.5 * cfg.search_width_deg:
                continue

            a_bin = int(floor(angle / cfg.angle_bin_deg)) % total_angle_bins
            r_bin = int(floor((r - cfg.min_range_m) / cfg.range_bin_m))
            key = (a_bin, r_bin)
            occupied.add(key)

            s = stats[key]
            s.point_count += 1
            s.r_min = min(s.r_min, r)
            s.r_max = max(s.r_max, r)
            s.z_min = min(s.z_min, z)
            s.z_max = max(s.z_max, z)

        for key in occupied:
            stats[key].frame_hits += 1

    if frame_count == 0:
        return PolarAnalysisResult(0, dict(stats), None)

    persistent_keys = {
        key for key, s in stats.items()
        if s.frame_hits / frame_count >= cfg.persistence_min
    }
    if not persistent_keys:
        return PolarAnalysisResult(frame_count, dict(stats), None)

    by_angle = defaultdict(list)
    for key in persistent_keys:
        by_angle[key[0]].append(key)

    counts = sorted(len(v) for v in by_angle.values())
    median_count = counts[len(counts) // 2] if counts else 0
    threshold = max(cfg.min_persistent_range_bins, median_count + 1)
    candidate_angles = [
        a for a, keys in by_angle.items()
        if len(keys) >= threshold
    ]
    if not candidate_angles:
        # Sparse pole-like structures can occupy only one or two range bins.
        threshold = cfg.min_persistent_range_bins
        candidate_angles = [
            a for a, keys in by_angle.items()
            if len(keys) >= threshold
        ]
    if not candidate_angles:
        return PolarAnalysisResult(frame_count, dict(stats), None)

    groups = _group_circular(candidate_angles, total_angle_bins)

    def group_score(group):
        keys = [key for a in group for key in by_angle[a]]
        return sum(stats[k].frame_hits / frame_count for k in keys)

    best_group = max(groups, key=group_score)
    best_keys = [key for a in best_group for key in by_angle[a]]
    angle_centers = [
        (a + 0.5) * cfg.angle_bin_deg % 360.0 for a in best_group
    ]
    angle_weights = [len(by_angle[a]) for a in best_group]
    center = _circular_mean_deg(angle_centers, angle_weights)
    width = min(360.0, len(best_group) * cfg.angle_bin_deg + 2.0 * cfg.angle_padding_deg)

    r_min = min(stats[k].r_min for k in best_keys)
    r_max = max(stats[k].r_max for k in best_keys)
    z_min = min(stats[k].z_min for k in best_keys)
    z_max = max(stats[k].z_max for k in best_keys)
    mean_persistence = fmean(stats[k].frame_hits / frame_count for k in best_keys)
    support_factor = min(1.0, len(best_group) * cfg.angle_bin_deg / 10.0)
    confidence = max(0.0, min(1.0, mean_persistence * support_factor))

    suggestion = SectorSuggestion(
        center_deg=center,
        width_deg=width,
        min_range_m=max(cfg.min_range_m, r_min - cfg.range_padding_m),
        max_range_m=min(cfg.max_range_m, r_max + cfg.range_padding_m),
        z_min_m=max(cfg.z_min_m, z_min - cfg.z_padding_m),
        z_max_m=min(cfg.z_max_m, z_max + cfg.z_padding_m),
        confidence=confidence,
        supporting_angle_bins=len(best_group),
        supporting_polar_bins=len(best_keys),
    )
    return PolarAnalysisResult(frame_count, dict(stats), suggestion)
