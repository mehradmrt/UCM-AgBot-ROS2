import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    # Declare and configure launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )


    # Define the path to the map file and the parameter file (if needed)
    map_file_path = os.path.join(get_package_share_directory('robot_bringup'), 'maps', 'orchard_irr.yaml')
    # print(map_file_path)
    # Define and configure the map_server node
    map_server =  Node(
            package='nav2_map_server',
            executable='map_server',
            name= 'map_server', 
            parameters=[{'yaml_filename': map_file_path},
                        {'topic_name':'/map'}],
            output='screen'
        )

    # Define and configure the lifecycle manager node
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

    # Return the launch description with all the nodes
    return LaunchDescription([
        use_sim_time,
        # map_path,
        map_server,
        lifecycle_manager
    ])
