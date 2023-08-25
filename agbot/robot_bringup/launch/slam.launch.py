import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    map_file_path = os.path.join(get_package_share_directory('robot_bringup'), 'maps', 'orchard_irr.yaml')
    rviz_file_path = os.path.join(get_package_share_directory('robot_bringup'), 'rviz', 'slam.rviz')
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{map_file_path,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file_path]
        ),
    ])
