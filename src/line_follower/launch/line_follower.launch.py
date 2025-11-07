from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('line_follower')
    map_yaml = os.path.join(pkg_share, 'maps', 'line_map.yaml')

    return LaunchDescription([
        Node(
            package='line_follower',
            executable='line_follower',
            name='line_follower',
            output='screen',
            parameters=[
                {'map_path': map_yaml},
                {'use_turtlesim': True},
                {'linear_speed': 0.8},
                {'angular_gain': 2.0},
                {'waypoint_tolerance': 0.3},
            ]
        )
    ])
