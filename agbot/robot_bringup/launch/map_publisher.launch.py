import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    map_file_path = os.path.join(get_package_share_directory('robot_bringup'), 'maps', 'orchard_irr.yaml')
    map_server =  Node(
            package='nav2_map_server',
            executable='map_server',
            name= 'map_server', 
            parameters=[{'yaml_filename': map_file_path}],
            output='screen',
            remappings=remappings,
        )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    return LaunchDescription([
        use_sim_time,
        map_server,
        lifecycle_manager
    ])
