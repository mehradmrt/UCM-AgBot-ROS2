import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    desc_pkg = get_package_share_path('robot_description')
    default_model_path = os.path.join(desc_pkg, 'urdf', 'robot_description.urdf.xacro')

    my_pkg_path = get_package_share_path('robot_bringup')
    default_ekf_path = os.path.join(my_pkg_path, 'config', 'ekf_params.yaml')
    default_rviz_config_path = os.path.join(my_pkg_path, 'rviz', 'localization.rviz')

    map_launch_path = os.path.join(my_pkg_path, 'launch', 'map_publisher.launch.py')

    nav_bringup_dir = get_package_share_path('nav2_bringup')
    nav_launch_dir = os.path.join(nav_bringup_dir, 'launch')

    sensor_launch_path = os.path.join(my_pkg_path, 'launch', 'sensors.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description = Command(['xacro ', LaunchConfiguration('model')])

    slam_pkg = get_package_share_path('slam_toolbox')
    slam_dir = os.path.join(slam_pkg, 'launch', 'online_async_launch.py')


    remappings = ('/tf', 'tf'),('/tf_static', 'tf_static')

    model_arg = DeclareLaunchArgument(
            'model',
            default_value=default_model_path,
            description='Path to the URDF file'
        )

    sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='use simulation time'
        )

    robot_velocity_pub_node = Node(
            package='robot_bringup',
            executable='robot_velocity_pub',
            name='robot_velocity_publisher',
        )

    navsat_transform_node = Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[default_ekf_path, {'use_sim_time': use_sim_time}],
            remappings=[('imu/data', 'vectornav/imu_uncompensated'),
                        ('gps/fix', 'gps/fix'), 
                        ('odometry/filtered', 'odometry/global'),
                        ('gps/filtered', 'gps/filtered'),
                        ('odometry/gps', 'odometry/gps'),remappings] 
        )

    ekf_filter_node_odom = Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_odom',
	        output='screen',
            parameters=[default_ekf_path, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/local'),
                        ('/set_pose', '/initialpose'),remappings]         
        )

    ekf_filter_node_map = Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_map',
	        output='screen',
            parameters=[default_ekf_path, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/global'),
                        ('/set_pose', '/initialpose'),remappings]    
           )  

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output= 'screen',
            parameters=[{'robot_description': robot_description ,
                        'use_sim_time' : use_sim_time}],
            remappings=[remappings]
        )

    joint_state_pub = Node(
            package='robot_bringup',
            executable='joint_state_pub',
            name='joint_state_publisher_custom',
            remappings=[remappings]
        )

    map_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_launch_path),
            launch_arguments={'use_sim_time': use_sim_time}.items())

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, 'rviz_launch.py')),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False',
                          'rviz_config': os.path.join(my_pkg_path, 'rviz', 'navigation.rviz'),}.items())
                        #   'rviz_config': os.path.join(nav_bringup_dir, 'rviz', 'nav2_default_view.rviz'),}.items())

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensor_launch_path))

    robot_control = Node(
            package='robot_control',
            executable='motor_controller',
            name='motor_controller_node',
        )
    
    slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_dir),
            launch_arguments={'use_sim_time': use_sim_time}.items())

    return LaunchDescription([
        sim_time_arg,
        model_arg,
        # robot_control,
        sensor_launch,
        # robot_velocity_pub_node,
        robot_state_publisher,
        joint_state_pub,
        # navsat_transform_node,
        # ekf_filter_node_odom,
        # ekf_filter_node_map,
        # map_server,
        rviz,
        slam

    ])



# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, Command
# from launch_ros.actions import Node
# from nav2_common.launch import RewrittenYaml
# from ament_index_python.packages import get_package_share_path

# def generate_launch_description():
#     # Input parameters declaration
#     namespace = LaunchConfiguration('namespace')
#     params_file = LaunchConfiguration('params_file')
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     autostart = LaunchConfiguration('autostart')

#     # My package
#     my_pkg_path = get_package_share_path('robot_bringup')
    
#     # Robot Desc
#     desc_pkg = get_package_share_path('robot_description')
#     default_model_path = os.path.join(desc_pkg, 'urdf', 'robot_description.urdf.xacro')
#     robot_description = Command(['xacro ', default_model_path])

#     # Nav2 path
#     nav_bringup_dir = get_package_share_path('nav2_bringup')
#     nav_launch_dir = os.path.join(nav_bringup_dir, 'launch')

#     #remapping
#     remapping = ('/tf', 'tf'),('/tf_static', 'tf_static')

#     # Variables
#     lifecycle_nodes = ['map_saver']

#     # Getting directories and launch-files
#     bringup_dir = get_package_share_directory('nav2_bringup')
#     slam_toolbox_dir = get_package_share_directory('slam_toolbox')
#     slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')

#     # Create our own temporary YAML files that include substitutions
#     param_substitutions = {
#         'use_sim_time': use_sim_time}

#     configured_params = RewrittenYaml(
#         source_file=params_file,
#         root_key=namespace,
#         param_rewrites=param_substitutions,
#         convert_types=True)

#     # Declare the launch arguments
#     declare_namespace_cmd = DeclareLaunchArgument(
#         'namespace',
#         default_value='',
#         description='Top-level namespace')

#     declare_params_file_cmd = DeclareLaunchArgument(
#         'params_file',
#         default_value=os.path.join(my_pkg_path, 'params', 'agbot.yaml'),
#         description='Full path to the ROS2 parameters file to use for all launched nodes')

#     declare_use_sim_time_cmd = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='false',
#         description='Use simulation (Gazebo) clock if true')

#     declare_autostart_cmd = DeclareLaunchArgument(
#         'autostart', default_value='true',
#         description='Automatically startup the nav2 stack')
    
#     # Nodes to start the robot
#     # robot_velocity_pub_node = Node(
#     #         package='robot_bringup',
#     #         executable='robot_velocity_pub',
#     #         name='robot_velocity_publisher',
#     #     )    
    
#     robot_state_publisher = Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             output= 'screen',
#             parameters=[{'robot_description': robot_description ,
#                         'use_sim_time' : use_sim_time}],
#             remappings=[remapping]
#         )

#     joint_state_pub = Node(
#             package='robot_bringup',
#             executable='joint_state_pub',
#             name='joint_state_publisher_custom',
#             remappings=[remapping]
#         )
    
#     robot_control = Node(
#             package='robot_control',
#             executable='motor_controller',
#             name='motor_controller_node',
#         )
    
#     rviz = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, 'rviz_launch.py')),
#         launch_arguments={'namespace': '',
#                           'use_namespace': 'False',
#                           'rviz_config': os.path.join(my_pkg_path, 'rviz', 'navigation.rviz'),}.items())
#                         #   'rviz_config': os.path.join(nav_bringup_dir, 'rviz', 'nav2_default_view.rviz'),}.items())

#     # Nodes launching commands
#     start_slam_toolbox_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(slam_launch_file),
#         launch_arguments={'namespace': namespace,
#                             'use_sim_time': use_sim_time,
#                             'autostart': autostart,
#                             'params_file': params_file}.items())

#     start_map_saver_server_cmd = Node(
#             package='nav2_map_server',
#             executable='map_saver_server',
#             output='screen',
#             parameters=[configured_params],
#             remappings=[remapping])

#     start_lifecycle_manager_cmd = Node(
#             package='nav2_lifecycle_manager',
#             executable='lifecycle_manager',
#             name='lifecycle_manager_slam',
#             output='screen',
#             parameters=[{'use_sim_time': use_sim_time},
#                         {'autostart': autostart},
#                         {'node_names': lifecycle_nodes}],)

#     ld = LaunchDescription()

#     # Declare the launch options
#     ld.add_action(declare_namespace_cmd)
#     ld.add_action(declare_params_file_cmd)
#     ld.add_action(declare_use_sim_time_cmd)
#     ld.add_action(declare_autostart_cmd)

#     # Added
#     ld.add_action(robot_state_publisher)
#     ld.add_action(joint_state_pub)
#     ld.add_action(rviz)
#     # ld.add_action(robot_control)

#     # Running SLAM Toolbox
#     ld.add_action(start_slam_toolbox_cmd)

#     # Running Map Saver Server
#     ld.add_action(start_map_saver_server_cmd)
#     ld.add_action(start_lifecycle_manager_cmd)



#     return ld