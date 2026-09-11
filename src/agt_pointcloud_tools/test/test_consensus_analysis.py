from agt_pointcloud_tools.consensus_analysis import (
    ConsensusConfig,
    RobotEnvelope,
    build_cross_segment_consensus,
)
from agt_pointcloud_tools.polar_analysis import BinStats, PolarAnalysisResult


def _result(key, hits, frames=10, z_min=0.5, z_max=1.0):
    return PolarAnalysisResult(
        frame_count=frames,
        bin_stats={
            key: BinStats(
                frame_hits=hits,
                point_count=hits,
                r_min=0.17,
                r_max=0.18,
                z_min=z_min,
                z_max=z_max,
            )
        },
        suggestion=None,
    )


def test_cross_segment_self_geometry_consensus():
    # 181 deg, 0.175 m: inside the default Bunker footprint.
    results = [_result((90, 2), 10) for _ in range(5)]
    cells = build_cross_segment_consensus(
        results,
        angle_bin_deg=2.0,
        min_range_m=0.05,
        range_bin_m=0.05,
        cfg=ConsensusConfig(
            within_segment_persistence_min=0.5,
            segment_support_min=0.6,
            robot=RobotEnvelope(),
        ),
    )
    assert len(cells) == 1
    assert cells[0].classification == 'self_geometry'
    assert cells[0].segment_support == 1.0


def test_external_persistent_is_not_self_geometry():
    # 271 deg, 1.075 m: well outside the robot footprint.
    results = [_result((135, 20), 10, z_min=0.5, z_max=1.8) for _ in range(5)]
    cells = build_cross_segment_consensus(
        results,
        angle_bin_deg=2.0,
        min_range_m=0.05,
        range_bin_m=0.05,
    )
    assert len(cells) == 1
    assert cells[0].classification == 'external_persistent'


def test_requires_support_across_segments():
    results = [
        _result((90, 2), 10),
        _result((90, 2), 10),
        PolarAnalysisResult(10, {}, None),
        PolarAnalysisResult(10, {}, None),
        PolarAnalysisResult(10, {}, None),
    ]
    cells = build_cross_segment_consensus(
        results,
        angle_bin_deg=2.0,
        min_range_m=0.05,
        range_bin_m=0.05,
        cfg=ConsensusConfig(segment_support_min=0.6),
    )
    assert cells == []
