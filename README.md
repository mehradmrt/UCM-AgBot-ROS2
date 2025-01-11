# RoMu4o: A Robotic Manipulation Unit For Orchard Operations Automating Proximal Hyperspectral Leaf Sensing



## Requirements

- [Ubuntu (20.04)](https://releases.ubuntu.com/20.04/)

- [ROS2 (Foxy)](https://docs.ros.org/en/foxy/Installation.html)  

- [MoveIt2 (Foxy)](https://moveit.ai/install-moveit2/binary/)


Forked Repositories : 

- [RealSense (D435i)](https://github.com/mehradmrt/realsense-ros)

- [TM/OMRON (TM5M-900)](https://github.com/mehradmrt/tmr_ros2)

- [Vectornav](https://github.com/mehradmrt/vectornav)


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

### RoMu4o Navigation

Launch driver packages for sensors and devices:

```sh
ros2 launch robot_bringup sensors.launch.py imu:=true gnss:=true lidar2d:=true realsense:=true encoders:=true
 ```

Launch robot bringup:

```sh
ros2 launch robot_bringup bringup.launch.py 
```

Launch robot localization:

```sh
ros2 launch robot_bringup localization.launch.py 
```

Launch robot navigation:

```sh
ros2 launch robot_bringup navigation.launch.py 
```



### RoMu4o Perception and Leaf Manipulation

Launch driver packages for sensors and devices:

```sh
ros2 launch robot_bringup sensors_arm.launch.py 
 ```

Launch the robotic manipulation pipeline:

```sh
ros2 launch tm_moveit_cpp_demo tm5-900_run_moveit_cpp.launch.py robot_ip:=192.168.1.19 
```

Run the perception pipeline:

```sh
ros2 run leaf_extraction instance_segmentation 
```


## Cite this work

coming soon...