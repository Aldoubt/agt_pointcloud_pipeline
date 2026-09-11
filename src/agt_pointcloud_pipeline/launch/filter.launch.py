from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    profile = LaunchConfiguration('profile').perform(context)
    share = Path(get_package_share_directory('agt_pointcloud_pipeline'))
    config = share / 'config' / 'profiles' / f'{profile}.yaml'
    if not config.exists():
        raise RuntimeError(f'unknown pointcloud profile: {profile} ({config})')

    return [
        Node(
            package='agt_pointcloud_pipeline',
            executable='filter_node',
            name='agt_pointcloud_filter',
            output='screen',
            parameters=[str(config)],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile',
            default_value='navigation',
            description='Profile: navigation, mapping, localization, or analysis'),
        OpaqueFunction(function=_launch),
    ])
