import argparse
import csv
from pathlib import Path

import yaml

from .bag_cloud import collect_cloud_frames_by_segments, discover_cloud_topic
from .bag_imu import read_imu_samples
from .consensus_analysis import (
    ConsensusConfig,
    RobotEnvelope,
    build_cross_segment_consensus,
)
from .polar_analysis import PolarAnalysisConfig, RigidTransform, analyze_polar_persistence
from .reporting import load_transform
from .static_detector import StaticDetectorConfig, detect_static_segments


def _parser():
    p = argparse.ArgumentParser(
        description='Cross-segment robot-relative persistence analysis for self geometry.')
    p.add_argument('--bag', required=True)
    p.add_argument('--imu-topic', default='/agt/sensors/imu/data')
    p.add_argument('--cloud-topic', default='auto')
    p.add_argument('--extrinsic-yaml')
    p.add_argument('--assume-source-is-robot-frame', action='store_true')
    p.add_argument('--output', default='consensus.yaml')
    p.add_argument('--max-segments', type=int, default=0)
    p.add_argument('--point-stride', type=int, default=1)
    p.add_argument('--max-frames-per-segment', type=int, default=100)

    p.add_argument('--angle-bin-deg', type=float, default=2.0)
    p.add_argument('--range-bin-m', type=float, default=0.05)
    p.add_argument('--analysis-min-range-m', type=float, default=0.05)
    p.add_argument('--analysis-max-range-m', type=float, default=1.50)
    p.add_argument('--analysis-z-min-m', type=float, default=-0.50)
    p.add_argument('--analysis-z-max-m', type=float, default=2.00)
    p.add_argument('--search-center-deg', type=float, default=180.0)
    p.add_argument('--search-width-deg', type=float, default=360.0)

    p.add_argument('--within-segment-persistence-min', type=float, default=0.50)
    p.add_argument('--segment-support-min', type=float, default=0.60)

    p.add_argument('--robot-length-m', type=float, default=1.023)
    p.add_argument('--robot-width-m', type=float, default=0.778)
    p.add_argument('--robot-z-min-m', type=float, default=-0.20)
    p.add_argument('--robot-z-max-m', type=float, default=1.50)
    p.add_argument('--robot-padding-m', type=float, default=0.05)

    p.add_argument('--window-sec', type=float, default=0.75)
    p.add_argument('--min-duration-sec', type=float, default=2.0)
    p.add_argument('--max-gyro-mean', type=float, default=0.06)
    p.add_argument('--max-gyro-std', type=float, default=0.025)
    p.add_argument('--max-acc-norm-std', type=float, default=0.15)
    return p


def main():
    args = _parser().parse_args()

    samples = read_imu_samples(args.bag, args.imu_topic)
    static_cfg = StaticDetectorConfig(
        window_sec=args.window_sec,
        min_duration_sec=args.min_duration_sec,
        max_gyro_mean=args.max_gyro_mean,
        max_gyro_std=args.max_gyro_std,
        max_acc_norm_std=args.max_acc_norm_std,
    )
    segments = detect_static_segments(samples, static_cfg)
    if args.max_segments > 0:
        segments = segments[:args.max_segments]
    if not segments:
        raise RuntimeError('No static segments found.')

    cloud_topic, cloud_type = (
        discover_cloud_topic(args.bag)
        if args.cloud_topic == 'auto'
        else (args.cloud_topic, 'explicit')
    )

    if args.extrinsic_yaml:
        transform = load_transform(args.extrinsic_yaml)
        transform_source = str(Path(args.extrinsic_yaml).expanduser().resolve())
    elif args.assume_source_is_robot_frame:
        transform = RigidTransform()
        transform_source = 'explicit_identity'
    else:
        raise RuntimeError(
            'Provide --extrinsic-yaml or --assume-source-is-robot-frame.')

    frames_by_segment = collect_cloud_frames_by_segments(
        args.bag,
        cloud_topic,
        segments,
        point_stride=args.point_stride,
        max_frames_per_segment=args.max_frames_per_segment,
    )

    polar_cfg = PolarAnalysisConfig(
        angle_bin_deg=args.angle_bin_deg,
        range_bin_m=args.range_bin_m,
        min_range_m=args.analysis_min_range_m,
        max_range_m=args.analysis_max_range_m,
        z_min_m=args.analysis_z_min_m,
        z_max_m=args.analysis_z_max_m,
        search_center_deg=args.search_center_deg,
        search_width_deg=args.search_width_deg,
        persistence_min=args.within_segment_persistence_min,
        min_persistent_range_bins=1,
    )

    results = []
    segment_rows = []
    for index, segment in enumerate(segments):
        frames = frames_by_segment.get(index, [])
        result = analyze_polar_persistence(frames, transform, polar_cfg)
        results.append(result)
        segment_rows.append({
            'index': index,
            'start_sec': round(segment.start, 6),
            'end_sec': round(segment.end, 6),
            'duration_sec': round(segment.end - segment.start, 6),
            'score': round(segment.score, 6),
            'frames_analyzed': result.frame_count,
        })

    consensus_cfg = ConsensusConfig(
        within_segment_persistence_min=args.within_segment_persistence_min,
        segment_support_min=args.segment_support_min,
        robot=RobotEnvelope(
            length_m=args.robot_length_m,
            width_m=args.robot_width_m,
            z_min_m=args.robot_z_min_m,
            z_max_m=args.robot_z_max_m,
            padding_m=args.robot_padding_m,
        ),
    )

    cells = build_cross_segment_consensus(
        results,
        angle_bin_deg=polar_cfg.angle_bin_deg,
        min_range_m=polar_cfg.min_range_m,
        range_bin_m=polar_cfg.range_bin_m,
        cfg=consensus_cfg,
    )

    out = Path(args.output).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    csv_path = out.with_name(out.stem + '_cells.csv')

    self_cells = [c for c in cells if c.classification == 'self_geometry']
    external_cells = [c for c in cells if c.classification == 'external_persistent']

    payload = {
        'bag': str(Path(args.bag).expanduser().resolve()),
        'imu_topic': args.imu_topic,
        'cloud_topic': cloud_topic,
        'cloud_type': cloud_type,
        'transform_source': transform_source,
        'source_to_robot': {
            'translation': list(transform.translation),
            'rpy_deg': list(transform.rpy_deg),
        },
        'segments': segment_rows,
        'consensus_config': {
            'within_segment_persistence_min': args.within_segment_persistence_min,
            'segment_support_min': args.segment_support_min,
            'angle_bin_deg': args.angle_bin_deg,
            'range_bin_m': args.range_bin_m,
            'robot_length_m': args.robot_length_m,
            'robot_width_m': args.robot_width_m,
            'robot_z_min_m': args.robot_z_min_m,
            'robot_z_max_m': args.robot_z_max_m,
            'robot_padding_m': args.robot_padding_m,
        },
        'summary': {
            'consensus_cells': len(cells),
            'self_geometry_cells': len(self_cells),
            'external_persistent_cells': len(external_cells),
        },
        'self_geometry_top': [
            {
                'angle_center_deg': round(c.angle_center_deg, 3),
                'range_center_m': round(c.range_center_m, 3),
                'segment_support': round(c.segment_support, 3),
                'mean_persistence': round(c.mean_persistence, 3),
                'z_min_m': round(c.z_min_m, 3),
                'z_max_m': round(c.z_max_m, 3),
            }
            for c in self_cells[:30]
        ],
    }
    out.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')

    with csv_path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            'classification', 'angle_center_deg', 'range_center_m',
            'segment_support', 'mean_persistence', 'supporting_segments',
            'total_segments', 'z_min_m', 'z_max_m',
        ])
        for c in cells:
            writer.writerow([
                c.classification,
                round(c.angle_center_deg, 6),
                round(c.range_center_m, 6),
                round(c.segment_support, 6),
                round(c.mean_persistence, 6),
                c.supporting_segments,
                c.total_segments,
                round(c.z_min_m, 6),
                round(c.z_max_m, 6),
            ])

    print(f'Static segments analyzed: {len(segments)}')
    print(f'Cloud topic: {cloud_topic}')
    print(f'Consensus cells: {len(cells)}')
    print(f'Self-geometry cells: {len(self_cells)}')
    print(f'External persistent cells: {len(external_cells)}')
    print(f'Report: {out}')
    print(f'Cells CSV: {csv_path}')
