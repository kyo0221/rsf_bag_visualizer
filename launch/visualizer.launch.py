import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_dir', default_value=os.path.expanduser('~/rsf_ws/rosbag')),
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('open_browser', default_value='true'),
        Node(
            package='rsf_bag_visualizer',
            executable='server',
            arguments=['--bag-dir', LaunchConfiguration('bag_dir'),
                       '--port', LaunchConfiguration('port'),
                       '--open-browser', LaunchConfiguration('open_browser')],
            output='screen'),
    ])
