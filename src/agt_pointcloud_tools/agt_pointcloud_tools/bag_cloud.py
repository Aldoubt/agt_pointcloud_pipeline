from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator, List, Optional, Sequence, Tuple

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2


@dataclass(frozen=True)
class CloudFrame:
    t: float
    frame_id: str
    points: Sequence[Tuple[float, float, float]]


SUPPORTED_POINTCLOUD_TYPES = {
    'sensor_msgs/msg/PointCloud2',
    'livox_ros_driver2/msg/CustomMsg',
}


def _open_reader(bag_path: str):
    path = str(Path(bag_path).expanduser().resolve())
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    return reader


def discover_cloud_topic(bag_path: str) -> Tuple[str, str]:
    reader = _open_reader(bag_path)
    type_map = {info.name: info.type for info in reader.get_all_topics_and_types()}
    preferred = (
        '/agt/livox/points',
        '/fastlio2/body_cloud',
        '/agt/sensors/lidar/custom',
        '/livox/lidar',
    )
    for topic in preferred:
        if type_map.get(topic) in SUPPORTED_POINTCLOUD_TYPES:
            return topic, type_map[topic]
    for topic, msg_type in sorted(type_map.items()):
        if msg_type in SUPPORTED_POINTCLOUD_TYPES:
            return topic, msg_type
    supported = ', '.join(sorted(SUPPORTED_POINTCLOUD_TYPES))
    raise RuntimeError(f'No supported cloud topic found. Supported types: {supported}')


def _extract_points(msg, msg_type: str, point_stride: int):
    stride = max(1, int(point_stride))
    if msg_type == 'sensor_msgs/msg/PointCloud2':
        out = []
        for i, p in enumerate(point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)):
            if i % stride == 0:
                out.append((float(p[0]), float(p[1]), float(p[2])))
        return out

    # Livox CustomMsg is loaded dynamically; no compile-time driver dependency.
    out = []
    for i, p in enumerate(msg.points):
        if i % stride == 0:
            out.append((float(p.x), float(p.y), float(p.z)))
    return out


def iter_cloud_frames(
    bag_path: str,
    topic: str,
    start_sec: float,
    end_sec: float,
    point_stride: int = 2,
    max_frames: int = 80,
) -> Iterator[CloudFrame]:
    if topic == 'auto':
        topic, _ = discover_cloud_topic(bag_path)

    reader = _open_reader(bag_path)
    type_map = {info.name: info.type for info in reader.get_all_topics_and_types()}
    if topic not in type_map:
        raise RuntimeError(f'Cloud topic {topic!r} not found in bag')
    msg_type_name = type_map[topic]
    if msg_type_name not in SUPPORTED_POINTCLOUD_TYPES:
        raise RuntimeError(
            f'Cloud topic {topic!r} has unsupported type {msg_type_name!r}')

    msg_type = get_message(msg_type_name)
    bag_t0 = None
    emitted = 0

    while reader.has_next():
        name, data, timestamp_ns = reader.read_next()
        t = timestamp_ns * 1e-9
        if bag_t0 is None:
            bag_t0 = t
        rel_t = t - bag_t0

        if rel_t > end_sec:
            break
        if name != topic or rel_t < start_sec:
            continue

        msg = deserialize_message(data, msg_type)
        frame_id = getattr(getattr(msg, 'header', None), 'frame_id', '') or ''
        points = _extract_points(msg, msg_type_name, point_stride)
        yield CloudFrame(rel_t, frame_id, points)

        emitted += 1
        if max_frames > 0 and emitted >= max_frames:
            break
