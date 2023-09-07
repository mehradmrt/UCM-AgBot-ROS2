import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_file_name = 'robot_description_gazebo.urdf.xacro'
    # urdf_file_name = 'waffle.urdf'
    # urdf_file_name = 'sam_bot_description.urdf'
    urdf_path = os.path.join(get_package_share_directory('robot_simulator'),'urdf_sim', urdf_file_name)

    robot_desc = Command(['xacro ', urdf_path])
    # with open(urdf_path, 'r') as infp:
    #     robot_desc = infp.read()
    
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            remappings=remappings,
            arguments=[robot_desc]
            )
    ])