from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    share = Path(get_package_share_directory('agt_pointcloud_pipeline'))
    profile = LaunchConfiguration('profile').perform(context)
    config = share / 'config' / 'profiles' / f'{profile}.yaml'
    rviz = share / 'config' / 'filter_debug.rviz'
    if not config.exists():
        raise RuntimeError(f'unknown pointcloud profile: {profile}')

    overrides = {
        'input_topic': LaunchConfiguration('input_topic'),
        'target_frame': LaunchConfiguration('target_frame'),
        'statistics_period_sec': 1.0,
        'filters.rear_sector.center_deg': LaunchConfiguration('rear_center_deg'),
        'filters.rear_sector.width_deg': LaunchConfiguration('rear_width_deg'),
        'filters.rear_sector.min_range_m': LaunchConfiguration('rear_min_range_m'),
        'filters.rear_sector.max_range_m': LaunchConfiguration('rear_max_range_m'),
        'filters.rear_sector.z_min_m': LaunchConfiguration('rear_z_min_m'),
        'filters.rear_sector.z_max_m': LaunchConfiguration('rear_z_max_m'),
    }

    return [
        Node(
            package='agt_pointcloud_pipeline',
            executable='filter_node',
            name='agt_pointcloud_filter',
            output='screen',
            parameters=[str(config), overrides],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='agt_pointcloud_filter_rviz',
            output='screen',
            arguments=['-d', str(rviz)],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='analysis'),
        DeclareLaunchArgument('input_topic', default_value='/agt/livox/points'),
        DeclareLaunchArgument('target_frame', default_value='base_link'),
        DeclareLaunchArgument('rear_center_deg', default_value='180.0'),
        DeclareLaunchArgument('rear_width_deg', default_value='10.0'),
        DeclareLaunchArgument('rear_min_range_m', default_value='0.2'),
        DeclareLaunchArgument('rear_max_range_m', default_value='2.0'),
        DeclareLaunchArgument('rear_z_min_m', default_value='-0.5'),
        DeclareLaunchArgument('rear_z_max_m', default_value='1.5'),
        OpaqueFunction(function=_launch),
    ])
