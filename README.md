# RoMu4o: A Robotic Manipulation Unit For Orchard Operations Automating Proximal Hyperspectral Leaf Sensing

## Requirements

- [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)

- [ROS2 (Foxy)](https://docs.ros.org/en/foxy/Installation.html)  

- [MoveIt2 (Foxy)](https://moveit.ai/install-moveit2/binary/)

## Installation

Clone the main repository along with all the submodules using:

1. **Create a ROS2 workspace**:
    ```sh
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src/
    ```

2. **Clone the repository**:
   
    ```sh
    git clone --recurse-submodules https://github.com/mehradmrt/UCM-AgBot-ROS2 
    ```

3. **Return to the workspace root**:
    ```sh
    cd ../
    ```

4. **Install ROS dependencies**:
    ```sh
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```

5. **Build the workspace**:
    ```sh
    colcon build 
    ```

6. **Source the ROS2 environment**:
    ```sh
    source /opt/ros/foxy/setup.bash
    source install/setup.bash
    ```

## Usage

Leaf Manipulation requires launching the realsense :

    ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true pointcloud.ordered_pc:=true depth_module.profile:=1280x720x30 rgb_camera.profile:=1280x720x30

    ros2 param set /camera/camera pointcloud.ordered_pc true

Run the perception pipeline:

    ros2 run leaf_extraction instance_segmentation

Run the robotic manipulation pipeline:

    ros2 launch tm_moveit_cpp_demo tm5-900_run_moveit_cpp.launch.py robot_ip:=192.168.1.19

## Cite this work

coming soon...