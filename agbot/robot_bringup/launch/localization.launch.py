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

    return LaunchDescription([
        sim_time_arg,
        model_arg,
        robot_control,
        # sensor_launch,
        robot_velocity_pub_node,
        robot_state_publisher,
        joint_state_pub,
        navsat_transform_node,
        ekf_filter_node_odom,
        ekf_filter_node_map,
        map_server,
        rviz

    ])
