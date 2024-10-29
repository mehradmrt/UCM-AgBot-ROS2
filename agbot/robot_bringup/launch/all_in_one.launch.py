from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Path to the robot_bringup package
    package_name = 'robot_bringup'
    package_share_directory = FindPackageShare(package_name).find(package_name)

    # Launch file paths
    sensors_launch = os.path.join(package_share_directory, 'launch', 'sensors.launch.py')
    localization_launch = os.path.join(package_share_directory, 'launch', 'localization.launch.py')
    navigation_launch = os.path.join(package_share_directory, 'launch', 'navigation.launch.py')

    # Define each IncludeLaunchDescription
    sensors_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensors_launch)
    )

    localization_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch)
    )

    navigation_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch)
    )

    # Return a LaunchDescription containing all three launch actions
    return LaunchDescription([
        sensors_launch_description,
        localization_launch_description,
        navigation_launch_description
    ])
