import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(get_package_share_directory('tb3_lane_follower'),
                               'config', 'lane_follower.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_cfg),
        DeclareLaunchArgument('enabled', default_value='true',
                              description='false = detect only, robot does not move'),
        Node(package='tb3_lane_follower', executable='lane_follower', name='lane_follower',
             output='screen',
             parameters=[LaunchConfiguration('params_file'),
                         {'enabled': LaunchConfiguration('enabled')}]),
    ])
