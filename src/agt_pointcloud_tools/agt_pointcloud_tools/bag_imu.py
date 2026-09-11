from pathlib import Path
from typing import List

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from .static_detector import ImuSample


def read_imu_samples(bag_path: str, topic: str) -> List[ImuSample]:
    path = str(Path(bag_path).expanduser().resolve())
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {
        info.name: info.type
        for info in reader.get_all_topics_and_types()
    }
    if topic not in type_map:
        available = ', '.join(sorted(type_map))
        raise RuntimeError(f'IMU topic {topic!r} not found. Available: {available}')

    msg_type = get_message(type_map[topic])
    if type_map[topic] != 'sensor_msgs/msg/Imu':
        raise RuntimeError(
            f'{topic} has type {type_map[topic]}, expected sensor_msgs/msg/Imu')

    samples = []
    t0 = None
    while reader.has_next():
        name, data, timestamp_ns = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, msg_type)
        t = timestamp_ns * 1e-9
        if t0 is None:
            t0 = t
        samples.append(ImuSample(
            t=t - t0,
            gx=float(msg.angular_velocity.x),
            gy=float(msg.angular_velocity.y),
            gz=float(msg.angular_velocity.z),
            ax=float(msg.linear_acceleration.x),
            ay=float(msg.linear_acceleration.y),
            az=float(msg.linear_acceleration.z),
        ))

    return samples
