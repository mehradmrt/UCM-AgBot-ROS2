import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # urdf_path = os.path.join(get_package_share_directory('robot_simulator'),'urdf_sim','robot_description_gazebo.urdf.xacro')
    # model_path = os.path.join(get_package_share_directory('robot_simulator'),'models','my_robot','waffle.sdf')
    world_path = os.path.join(get_package_share_directory('robot_simulator'),'worlds','orchard.world')
    ekf_config_path = os.path.join(get_package_share_directory('robot_simulator'),'config','ekf.yaml')
    state_pub_path = os.path.join(get_package_share_directory('robot_simulator'),'launch','myrobot_state_publisher.launch.py')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    # robot_desc = Command(['xacro ', urdf_path]) 
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='27.0') 
    y_pose = LaunchConfiguration('y_pose', default='-21.0')
    Y_pose = LaunchConfiguration('Y_pose', default='1.5707')



    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(state_pub_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(PythonExpression(['True', ' and not ', 'False']))
    )

    gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            # '-file', model_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z','0.0',
            '-Y', Y_pose,
            '-topic', 'robot_description'
        ],
        output='screen',
    )

    robot_localization_node = Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
         remappings=[('/set_pose', '/initialpose')]
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(gazebo_spawner_cmd)
    ld.add_action(robot_localization_node)
    

    return ld
