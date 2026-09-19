from math import nan
from types import SimpleNamespace

from agt_pointcloud_tools.bag_cloud import _extract_points, _valid_xyz


def _point(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z)


def test_zero_and_nonfinite_xyz_are_invalid():
    assert not _valid_xyz(0.0, 0.0, 0.0)
    assert not _valid_xyz(nan, 0.0, 0.0)
    assert _valid_xyz(0.1, 0.0, 0.0)


def test_custommsg_zero_returns_do_not_consume_stride_slots():
    msg = SimpleNamespace(points=[
        _point(0.0, 0.0, 0.0),
        _point(1.0, 0.0, 0.0),
        _point(0.0, 0.0, 0.0),
        _point(2.0, 0.0, 0.0),
        _point(3.0, 0.0, 0.0),
    ])

    points = _extract_points(msg, 'livox_ros_driver2/msg/CustomMsg', 2)

    assert points == [(1.0, 0.0, 0.0), (3.0, 0.0, 0.0)]
