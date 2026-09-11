import csv
from pathlib import Path
from typing import Mapping, Tuple

import yaml

from .polar_analysis import PolarAnalysisConfig, PolarAnalysisResult, RigidTransform


def load_transform(path: str) -> RigidTransform:
    data = yaml.safe_load(Path(path).expanduser().read_text(encoding='utf-8')) or {}
    node = data.get('source_to_robot', data)
    translation = tuple(float(v) for v in node.get('translation', [0.0, 0.0, 0.0]))
    rpy_deg = tuple(float(v) for v in node.get('rpy_deg', [0.0, 0.0, 0.0]))
    if len(translation) != 3 or len(rpy_deg) != 3:
        raise RuntimeError('extrinsic YAML requires three translation and three rpy_deg values')
    return RigidTransform(translation=translation, rpy_deg=rpy_deg)


def write_polar_csv(path: Path, result: PolarAnalysisResult, cfg: PolarAnalysisConfig):
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            'angle_center_deg', 'range_center_m', 'persistence',
            'frame_hits', 'point_count', 'z_min_m', 'z_max_m',
        ])
        for (a_bin, r_bin), s in sorted(result.bin_stats.items()):
            writer.writerow([
                round((a_bin + 0.5) * cfg.angle_bin_deg, 6),
                round(cfg.min_range_m + (r_bin + 0.5) * cfg.range_bin_m, 6),
                round(s.frame_hits / max(result.frame_count, 1), 6),
                s.frame_hits,
                s.point_count,
                round(s.z_min, 6),
                round(s.z_max, 6),
            ])


def write_suggestion_yaml(path: Path, result: PolarAnalysisResult):
    if result.suggestion is None:
        payload = {
            'status': 'no_candidate',
            'message': 'No persistent rear interference sector satisfied the configured thresholds.',
        }
    else:
        s = result.suggestion
        payload = {
            'status': 'candidate',
            'confidence': round(s.confidence, 6),
            'runtime_parameter_snippet': {
                'filters.rear_sector.type':
                    'agt_pointcloud_pipeline/SectorFilterPlugin',
                'filters.rear_sector.center_deg': round(s.center_deg, 6),
                'filters.rear_sector.width_deg': round(s.width_deg, 6),
                'filters.rear_sector.min_range_m': round(s.min_range_m, 6),
                'filters.rear_sector.max_range_m': round(s.max_range_m, 6),
                'filters.rear_sector.z_min_m': round(s.z_min_m, 6),
                'filters.rear_sector.z_max_m': round(s.z_max_m, 6),
            },
            'support': {
                'angle_bins': s.supporting_angle_bins,
                'polar_bins': s.supporting_polar_bins,
            },
        }
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')
