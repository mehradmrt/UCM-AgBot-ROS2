from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    my_pkg = get_package_share_path('robot_bringup')
    default_model_path = my_pkg / 'urdf/robot_description.urdf'
    default_rviz_config_path = my_pkg / 'rviz/urdf.rviz'
    default_ekf_path = my_pkg / 'config/ekf_params.yaml'

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),value_type=str)


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

        DeclareLaunchArgument(
            name='gui', default_value='true', choices=['true', 'false'],
            description='Flag to enable joint_state_publisher_gui'),

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
            # remappings=[('imu/data', 'vectornav/imu')]
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
        output= 'screen',
        parameters=[{'robot_description': robot_description}]
        ),
        
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     # condition=UnlessCondition(LaunchConfiguration('gui'))
        # ),

        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     condition=IfCondition(LaunchConfiguration('gui'))
        # )

        Node(
            package='robot_bringup',
            executable='robot_state_publisher_costum',
            name='robot_state_publisher_costum',
        ),

    ])
