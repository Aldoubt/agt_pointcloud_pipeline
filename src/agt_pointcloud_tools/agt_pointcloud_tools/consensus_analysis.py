from dataclasses import dataclass
from math import cos, radians, sin
from statistics import fmean
from typing import Dict, Iterable, List, Sequence, Tuple

from .polar_analysis import PolarAnalysisResult


@dataclass(frozen=True)
class RobotEnvelope:
    length_m: float = 1.023
    width_m: float = 0.778
    z_min_m: float = -0.20
    z_max_m: float = 1.50
    padding_m: float = 0.05


@dataclass(frozen=True)
class ConsensusConfig:
    within_segment_persistence_min: float = 0.50
    segment_support_min: float = 0.60
    robot: RobotEnvelope = RobotEnvelope()


@dataclass(frozen=True)
class ConsensusCell:
    angle_center_deg: float
    range_center_m: float
    segment_support: float
    mean_persistence: float
    supporting_segments: int
    total_segments: int
    z_min_m: float
    z_max_m: float
    classification: str


def _classification(angle_deg: float, range_m: float, z_min: float, z_max: float,
                    robot: RobotEnvelope) -> str:
    a = radians(angle_deg)
    x = range_m * cos(a)
    y = range_m * sin(a)
    half_l = 0.5 * robot.length_m + robot.padding_m
    half_w = 0.5 * robot.width_m + robot.padding_m
    xy_inside = -half_l <= x <= half_l and -half_w <= y <= half_w
    z_overlap = z_max >= robot.z_min_m and z_min <= robot.z_max_m
    return 'self_geometry' if xy_inside and z_overlap else 'external_persistent'


def build_cross_segment_consensus(
    results: Sequence[PolarAnalysisResult],
    angle_bin_deg: float,
    min_range_m: float,
    range_bin_m: float,
    cfg: ConsensusConfig = ConsensusConfig(),
) -> List[ConsensusCell]:
    if not results:
        return []

    per_key: Dict[Tuple[int, int], List[Tuple[float, float, float]]] = {}

    for result in results:
        if result.frame_count <= 0:
            continue
        for key, stats in result.bin_stats.items():
            persistence = stats.frame_hits / result.frame_count
            if persistence < cfg.within_segment_persistence_min:
                continue
            per_key.setdefault(key, []).append(
                (persistence, stats.z_min, stats.z_max))

    total = len(results)
    cells = []
    for (a_bin, r_bin), observations in per_key.items():
        support = len(observations) / total
        if support < cfg.segment_support_min:
            continue

        angle = (a_bin + 0.5) * angle_bin_deg % 360.0
        range_m = min_range_m + (r_bin + 0.5) * range_bin_m
        z_min = min(v[1] for v in observations)
        z_max = max(v[2] for v in observations)

        cells.append(ConsensusCell(
            angle_center_deg=angle,
            range_center_m=range_m,
            segment_support=support,
            mean_persistence=fmean(v[0] for v in observations),
            supporting_segments=len(observations),
            total_segments=total,
            z_min_m=z_min,
            z_max_m=z_max,
            classification=_classification(angle, range_m, z_min, z_max, cfg.robot),
        ))

    return sorted(
        cells,
        key=lambda c: (
            0 if c.classification == 'self_geometry' else 1,
            -c.segment_support,
            -c.mean_persistence,
            c.range_center_m,
            c.angle_center_deg,
        ),
    )
