from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    imu_launch_dir = os.path.join(get_package_share_directory('vectornav'),'launch','vectornav.launch.py')
    lidar2d_launch_dir = os.path.join(get_package_share_directory('sllidar_ros2'),'launch','sllidar_s1_launch.py')
    realsense_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'),'launch','rs_launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    imu = DeclareLaunchArgument('imu', default_value='true', description='IMU sensor node')
    gnss = DeclareLaunchArgument('gnss', default_value='true', description='RTK/GNSS sensor node')
    lidar2d = DeclareLaunchArgument('lidar2d', default_value='true', description='Lidar2D sensor node')
    realsense = DeclareLaunchArgument('realsense', default_value='false', description='Realsense d435i sensor node')

    launch_actions = [
        imu,
        gnss,
        lidar2d,
        realsense,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch_dir),
            condition=IfCondition(LaunchConfiguration('imu'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar2d_launch_dir),
            condition=IfCondition(LaunchConfiguration('lidar2d'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_dir),
            condition=IfCondition(LaunchConfiguration('realsense'))
        ),
        Node(
            package='rtk_gnss',
            executable='RTK_GNSS',
            condition=IfCondition(LaunchConfiguration('gnss')),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ]

    return LaunchDescription(launch_actions)
