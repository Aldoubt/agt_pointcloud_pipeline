import argparse
from pathlib import Path

import yaml

from .bag_cloud import discover_cloud_topic, iter_cloud_frames
from .bag_imu import read_imu_samples
from .polar_analysis import (
    PolarAnalysisConfig,
    RigidTransform,
    analyze_polar_persistence,
)
from .reporting import load_transform, write_polar_csv, write_suggestion_yaml
from .static_detector import StaticDetectorConfig, detect_static_segments


def _parser():
    p = argparse.ArgumentParser(
        description='Analyze rosbag IMU + LiDAR data for point-cloud filter tuning.')
    p.add_argument('--bag', required=True)
    p.add_argument('--imu-topic', default='/agt/sensors/imu/data')
    p.add_argument('--cloud-topic', default='auto')
    p.add_argument('--output', default='analysis_report.yaml')
    p.add_argument('--static-only', action='store_true')

    p.add_argument('--window-sec', type=float, default=0.75)
    p.add_argument('--min-duration-sec', type=float, default=2.0)
    p.add_argument('--max-gyro-mean', type=float, default=0.06)
    p.add_argument('--max-gyro-std', type=float, default=0.025)
    p.add_argument('--max-acc-norm-std', type=float, default=0.15)
    p.add_argument('--segment-index', type=int, default=0)

    p.add_argument('--extrinsic-yaml')
    p.add_argument('--assume-source-is-robot-frame', action='store_true')
    p.add_argument('--point-stride', type=int, default=2)
    p.add_argument('--max-frames', type=int, default=80)

    p.add_argument('--angle-bin-deg', type=float, default=2.0)
    p.add_argument('--range-bin-m', type=float, default=0.10)
    p.add_argument('--analysis-min-range-m', type=float, default=0.20)
    p.add_argument('--analysis-max-range-m', type=float, default=2.00)
    p.add_argument('--analysis-z-min-m', type=float, default=-0.20)
    p.add_argument('--analysis-z-max-m', type=float, default=1.50)
    p.add_argument('--search-center-deg', type=float, default=180.0)
    p.add_argument('--search-width-deg', type=float, default=120.0)
    p.add_argument('--persistence-min', type=float, default=0.80)
    p.add_argument('--min-persistent-range-bins', type=int, default=2)
    return p


def main():
    args = _parser().parse_args()
    static_cfg = StaticDetectorConfig(
        window_sec=args.window_sec,
        min_duration_sec=args.min_duration_sec,
        max_gyro_mean=args.max_gyro_mean,
        max_gyro_std=args.max_gyro_std,
        max_acc_norm_std=args.max_acc_norm_std,
    )
    samples = read_imu_samples(args.bag, args.imu_topic)
    segments = detect_static_segments(samples, static_cfg)

    payload = {
        'bag': str(Path(args.bag).expanduser().resolve()),
        'imu_topic': args.imu_topic,
        'imu_sample_count': len(samples),
        'static_segments': [
            {
                'start_sec': round(s.start, 6),
                'end_sec': round(s.end, 6),
                'duration_sec': round(s.end - s.start, 6),
                'score': round(s.score, 6),
            }
            for s in segments
        ],
    }

    out = Path(args.output).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)

    if args.static_only:
        out.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')
        print(f'IMU samples: {len(samples)}')
        print(f'Static segments: {len(segments)}')
        print(f'Report: {out}')
        return

    if not segments:
        raise RuntimeError('No static segment found; tune IMU thresholds or use --static-only')
    if args.segment_index < 0 or args.segment_index >= len(segments):
        raise RuntimeError(
            f'--segment-index {args.segment_index} outside 0..{len(segments)-1}')
    segment = segments[args.segment_index]

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
            'Point analysis requires --extrinsic-yaml or '
            '--assume-source-is-robot-frame. Do not silently analyze a tilted LiDAR '
            'as if it were base_link.')

    polar_cfg = PolarAnalysisConfig(
        angle_bin_deg=args.angle_bin_deg,
        range_bin_m=args.range_bin_m,
        min_range_m=args.analysis_min_range_m,
        max_range_m=args.analysis_max_range_m,
        z_min_m=args.analysis_z_min_m,
        z_max_m=args.analysis_z_max_m,
        search_center_deg=args.search_center_deg,
        search_width_deg=args.search_width_deg,
        persistence_min=args.persistence_min,
        min_persistent_range_bins=args.min_persistent_range_bins,
    )

    frames = iter_cloud_frames(
        args.bag,
        cloud_topic,
        segment.start,
        segment.end,
        point_stride=args.point_stride,
        max_frames=args.max_frames,
    )
    result = analyze_polar_persistence(frames, transform, polar_cfg)

    payload['selected_static_segment'] = {
        'index': args.segment_index,
        'start_sec': round(segment.start, 6),
        'end_sec': round(segment.end, 6),
        'duration_sec': round(segment.end - segment.start, 6),
        'score': round(segment.score, 6),
    }
    payload['pointcloud_analysis'] = {
        'cloud_topic': cloud_topic,
        'cloud_type': cloud_type,
        'frames_analyzed': result.frame_count,
        'point_stride': args.point_stride,
        'transform_source': transform_source,
        'source_to_robot': {
            'translation': list(transform.translation),
            'rpy_deg': list(transform.rpy_deg),
        },
        'polar_config': {
            'angle_bin_deg': polar_cfg.angle_bin_deg,
            'range_bin_m': polar_cfg.range_bin_m,
            'min_range_m': polar_cfg.min_range_m,
            'max_range_m': polar_cfg.max_range_m,
            'z_min_m': polar_cfg.z_min_m,
            'z_max_m': polar_cfg.z_max_m,
            'search_center_deg': polar_cfg.search_center_deg,
            'search_width_deg': polar_cfg.search_width_deg,
            'persistence_min': polar_cfg.persistence_min,
        },
    }
    if result.suggestion:
        s = result.suggestion
        payload['pointcloud_analysis']['suggested_sector'] = {
            'center_deg': round(s.center_deg, 6),
            'width_deg': round(s.width_deg, 6),
            'min_range_m': round(s.min_range_m, 6),
            'max_range_m': round(s.max_range_m, 6),
            'z_min_m': round(s.z_min_m, 6),
            'z_max_m': round(s.z_max_m, 6),
            'confidence': round(s.confidence, 6),
        }
    else:
        payload['pointcloud_analysis']['suggested_sector'] = None

    out.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')
    csv_path = out.with_name(out.stem + '_polar.csv')
    suggestion_path = out.with_name(out.stem + '_suggested_filter.yaml')
    write_polar_csv(csv_path, result, polar_cfg)
    write_suggestion_yaml(suggestion_path, result)

    print(f'IMU samples: {len(samples)}')
    print(f'Static segments: {len(segments)}')
    print(
        f'Using segment {args.segment_index}: '
        f'{segment.start:.3f}s -> {segment.end:.3f}s')
    print(f'Cloud topic: {cloud_topic}')
    print(f'Frames analyzed: {result.frame_count}')
    if result.suggestion:
        s = result.suggestion
        print(
            'Suggested rear sector: '
            f'center={s.center_deg:.1f}deg width={s.width_deg:.1f}deg '
            f'range={s.min_range_m:.2f}-{s.max_range_m:.2f}m '
            f'z={s.z_min_m:.2f}-{s.z_max_m:.2f}m '
            f'confidence={s.confidence:.2f}')
    else:
        print('No persistent sector candidate found.')
    print(f'Report: {out}')
    print(f'Polar CSV: {csv_path}')
    print(f'Runtime snippet: {suggestion_path}')
