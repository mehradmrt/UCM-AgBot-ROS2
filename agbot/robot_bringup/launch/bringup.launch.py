from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
import os

def generate_launch_description():
    my_pkg = get_package_share_path('robot_bringup')
    default_model_path = my_pkg / 'urdf/robot_description.urdf'
    default_rviz_config_path = my_pkg / 'rviz/urdf.rviz'
    default_ekf_path = my_pkg / 'config/ekf_params.yaml'
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description = Command(['xacro ', str(default_model_path)])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=str(default_model_path),
            description='Path to the URDF file'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(default_rviz_config_path)],
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[default_ekf_path, {'use_sim_time': use_sim_time}],
            remappings=[('imu/data', 'vectornav/imu')]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            parameters=[default_ekf_path, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description},{'use_sim_time': use_sim_time}],
        ),


    ])
