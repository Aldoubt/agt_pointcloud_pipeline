from agt_pointcloud_tools.bag_cloud import CloudFrame
from agt_pointcloud_tools.polar_analysis import (
    PolarAnalysisConfig,
    RigidTransform,
    analyze_polar_persistence,
)


def test_detects_persistent_rear_cluster():
    frames = []
    for i in range(20):
        points = [
            (-0.8, -0.03, 0.2),
            (-0.9, 0.00, 0.5),
            (-1.0, 0.03, 0.8),
            (2.0, 0.0, 0.2),  # outside rear search window
        ]
        frames.append(CloudFrame(float(i), 'base_link', points))

    result = analyze_polar_persistence(
        frames,
        RigidTransform(),
        PolarAnalysisConfig(
            angle_bin_deg=5.0,
            range_bin_m=0.1,
            min_range_m=0.2,
            max_range_m=1.5,
            persistence_min=0.8,
            min_persistent_range_bins=2,
        ),
    )
    assert result.suggestion is not None
    assert abs(result.suggestion.center_deg - 180.0) < 10.0
    assert result.suggestion.min_range_m < 0.9
    assert result.suggestion.max_range_m > 0.9


def test_no_candidate_when_not_persistent():
    frames = [
        CloudFrame(0.0, 'base_link', [(-0.8, 0.0, 0.2)]),
        *[CloudFrame(float(i), 'base_link', []) for i in range(1, 10)],
    ]
    result = analyze_polar_persistence(
        frames,
        RigidTransform(),
        PolarAnalysisConfig(persistence_min=0.8),
    )
    assert result.suggestion is None



def test_single_persistent_polar_bin_is_not_enough():
    frames = [
        CloudFrame(float(i), 'base_link', [(-0.8, 0.0, 0.2)])
        for i in range(20)
    ]
    result = analyze_polar_persistence(
        frames,
        RigidTransform(),
        PolarAnalysisConfig(
            angle_bin_deg=5.0,
            range_bin_m=0.1,
            min_range_m=0.2,
            max_range_m=1.5,
            persistence_min=0.8,
            min_persistent_range_bins=2,
        ),
    )
    assert result.suggestion is None
