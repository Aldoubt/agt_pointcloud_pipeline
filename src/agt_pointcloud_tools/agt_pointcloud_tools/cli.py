import argparse
from pathlib import Path

import yaml

from .bag_imu import read_imu_samples
from .static_detector import StaticDetectorConfig, detect_static_segments


def _parser():
    p = argparse.ArgumentParser(
        description='Analyze a rosbag and find stable IMU intervals for LiDAR filter tuning.')
    p.add_argument('--bag', required=True)
    p.add_argument('--imu-topic', default='/agt/sensors/imu/data')
    p.add_argument('--output', default='static_segments.yaml')
    p.add_argument('--window-sec', type=float, default=0.75)
    p.add_argument('--min-duration-sec', type=float, default=2.0)
    p.add_argument('--max-gyro-mean', type=float, default=0.06)
    p.add_argument('--max-gyro-std', type=float, default=0.025)
    p.add_argument('--max-acc-norm-std', type=float, default=0.15)
    return p


def main():
    args = _parser().parse_args()
    cfg = StaticDetectorConfig(
        window_sec=args.window_sec,
        min_duration_sec=args.min_duration_sec,
        max_gyro_mean=args.max_gyro_mean,
        max_gyro_std=args.max_gyro_std,
        max_acc_norm_std=args.max_acc_norm_std,
    )
    samples = read_imu_samples(args.bag, args.imu_topic)
    segments = detect_static_segments(samples, cfg)

    payload = {
        'bag': str(Path(args.bag).expanduser().resolve()),
        'imu_topic': args.imu_topic,
        'sample_count': len(samples),
        'detector': {
            'window_sec': cfg.window_sec,
            'min_duration_sec': cfg.min_duration_sec,
            'max_gyro_mean': cfg.max_gyro_mean,
            'max_gyro_std': cfg.max_gyro_std,
            'max_acc_norm_std': cfg.max_acc_norm_std,
        },
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
    out.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')

    print(f'IMU samples: {len(samples)}')
    print(f'Static segments: {len(segments)}')
    for i, s in enumerate(segments[:10], 1):
        print(f'  {i}: {s.start:.3f}s -> {s.end:.3f}s '
              f'({s.end - s.start:.3f}s, score={s.score:.3f})')
    print(f'Report: {out}')
