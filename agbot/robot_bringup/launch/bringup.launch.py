import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    desc_pkg = get_package_share_path('robot_description')
    default_model_path = os.path.join(desc_pkg, 'urdf', 'robot_description.urdf.xacro')

    my_pkg_path = get_package_share_path('robot_bringup')
    default_ekf_path = os.path.join(my_pkg_path, 'config', 'ekf_params.yaml')
    default_rviz_config_path = os.path.join(my_pkg_path, 'rviz', 'localization.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description = Command(['xacro ', LaunchConfiguration('model')])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_model_path,
            description='Path to the URDF file'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='use simulation time'
        ),

        Node(
            package='robot_bringup',
            executable='robot_velocity_pub',
            name='robot_velocity_publisher',
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[default_ekf_path, {'use_sim_time': use_sim_time}],
            remappings=[('imu/data', 'vectornav/imu_uncompensated'),
                        ('gps/fix', 'gps/fix'), 
                        ('gps/filtered', 'gps/filtered'),
                        ('odometry/gps', 'odometry/gps'),
                        ('odometry/filtered', 'odometry/global')] 
        ),

        Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_odom',
	        output='screen',
            parameters=[default_ekf_path, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/local'),
                        ('/set_pose', '/initialpose')]         
        ),

        Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_map',
	        output='screen',
            parameters=[default_ekf_path, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/global'),
                        ('/set_pose', '/initialpose')]    
           ),     

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output= 'screen',
            parameters=[{'robot_description': robot_description ,
                        'use_sim_time' : use_sim_time}]
        ),

        Node(
            package='robot_bringup',
            executable='joint_state_pub',
            name='joint_state_publisher_custom',
        ),
        
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher'
        # ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', default_rviz_config_path],
        ),
    ])
