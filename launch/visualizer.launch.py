import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_dir', default_value=os.path.expanduser('~/rsf_ws/rosbag')),
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('open_browser', default_value='true'),
        ExecuteProcess(
            cmd=['ros2', 'run', 'rsf_bag_visualizer', 'server',
                 '--bag-dir', LaunchConfiguration('bag_dir'),
                 '--port', LaunchConfiguration('port'),
                 '--open-browser', LaunchConfiguration('open_browser')],
            output='screen'),
    ])
