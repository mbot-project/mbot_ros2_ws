# mbot_ros2_ws
> mbot - ROS2 Jazzy - Ubuntu 24.04

## To get started
### Clone the repository
This repository should be the `src` folder of the ROS2 workspace. So first we create the workspace `mbot_ws`. Then clone this repo as `src`.
```bash
mkdir mbot_ws
cd mbot_ws
git clone --recurse-submodules https://github.com/mbot-project/mbot_ros2_ws.git src
```
**OR** if you are using `gh`:
```bash
mkdir mbot_ws
cd mbot_ws
gh repo clone mbot-project/mbot_ros2_ws src
git submodule update --init --recursive
```

Then source the setup file:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> ~/.bashrc
```

### Build 
```bash
source /opt/ros/jazzy/setup.bash
cd ~/mbot_ws
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera
colcon build --symlink-install
echo "source $PWD/install/local_setup.bash" >> ~/.bashrc
source install/local_setup.bash
```
By default this will create the following directories as peers of the `src` directory:
- The `build` directory will be where intermediate files are stored. For each package a subfolder will be created in which e.g. CMake is being invoked.
- The `install` directory is where each package will be installed to. By default each package will be installed into a separate subdirectory.
- The `log` directory contains various logging information about each colcon invocation.

### Launch
#### sllidar_ros2
- ROS LiDAR driver
- To launch the lidar
    ```bash
    ros2 launch sllidar_ros2 sllidar_a1_launch.py frame_id:=lidar_link 
    ```
    - This will publish the topic `/scan`
#### mbot_description
- This is where we store urdf files.
- To launch the robot model and tf frames, and visualize it in the Rviz
    ```bash
    ros2 launch mbot_description rviz_launch.py
    ```
    - If you have launched the lidar, you can also visualize lidar scan on Rviz

#### mbot_navigation
- ROS package consists of slam_toolbox and nav2
- To launch slam_toolbox with mbot tailored config:
    ```bash
    ros2 launch mbot_navigation online_async_launch.py
    ```
    - This will publish the topic `/map`

#### mbot_vision
- ROS package for camera interface, image rectification, and AprilTag detection
- To launch camera only:
    ```bash
    ros2 launch mbot_vision camera.launch.py
    ```
    - This will publish topics `/camera/image_raw` and `/camera/camera_info`
- To launch camera with image rectification:
    ```bash
    ros2 launch mbot_vision camera_rectify.launch.py
    ```
    - This will also publish the topic `/image_rect`
- To launch full vision pipeline (camera + rectification + AprilTag detection):
    ```bash
    ros2 launch mbot_vision apriltag.launch.py
    ```
    - This will also publish AprilTag detection results on `/detections`


### Test
To test the mapping feature, you can drive the robot using keyboard:
```bash
# Run teleop node to drive robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Other packages
#### mbot_interfaces
- Where mbot custom messages and services are defined.
- The messages defined here are used on both Pi5 and Pico.