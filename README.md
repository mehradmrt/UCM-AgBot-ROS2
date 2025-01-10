# UCM-AgBot-ROS2

## Installation

### Repositories
Clone the main repository along with all the submodules using:

```sh
git clone --recurse-submodules https://github.com/mehradmrt/UCM-AgBot-ROS2 
```

### ROS2

### MoveIt2

## Usage

Camera:

    ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true pointcloud.ordered_pc:=true depth_module.profile:=1280x720x30 rgb_camera.profile:=1280x720x30

    ros2 param set /camera/camera pointcloud.ordered_pc true

    ros2 run leaf_extraction instance_segmentation

TM:

    ros2 launch tm_moveit_cpp_demo tm5-900_run_moveit_cpp.launch.py robot_ip:=192.168.1.19

## Cite this work