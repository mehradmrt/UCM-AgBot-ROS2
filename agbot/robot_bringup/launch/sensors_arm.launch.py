from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    realsense_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
    rg2_launch_dir = os.path.join(get_package_share_directory('onrobot_rg_control'), 'launch', 'bringup_server.launch.py')

    realsense_arg = DeclareLaunchArgument('realsense', default_value='false', description='Realsense d435i sensor node')
    rg2_arg = DeclareLaunchArgument('rg2', default_value='true', description='RG2 Gripper launch file')
    nanospec_arg = DeclareLaunchArgument('nanospec', default_value='true', description='Nanolambda nanospectrometer launch file')

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_dir),
        condition=IfCondition(LaunchConfiguration('realsense')),
        launch_arguments={
            'pointcloud.enable': 'true',
            'pointcloud.ordered_pc': 'true',
            'depth_module.profile': '1280x720x30',
            'rgb_camera.profile': '1280x720x30'
        }.items()
    )

    set_camera_param = TimerAction(
        period=1.0, 
        actions=[ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/camera/camera', 'pointcloud.ordered_pc', 'true'],
            shell=True
        )
    ],condition=IfCondition(LaunchConfiguration('realsense'))
    )

    rg2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rg2_launch_dir),
            condition=IfCondition(LaunchConfiguration('rg2'))
        )
    
    nanospec = Node(
            package='nanospec',
            executable='NSP32_service_node',
            condition=IfCondition(LaunchConfiguration('nanospec'))
        )

    return LaunchDescription([
        realsense_arg,
        rg2_arg,
        nanospec_arg,

        rg2,
        nanospec,
        realsense,

        set_camera_param  
    ])
 