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

## Launch
### Bring up the mbot
```bash
ros2 launch mbot_bringup mbot_bringup.launch.py 
```
- This will bring up the lidar driver, the robot description for visualization, and the tf tree.

### Visualize the robot model
```bash
ros2 launch mbot_bringup mbot_bringup.launch.py 
```
Run the following line **on NoMachine**:
```bash
ros2 launch mbot_bringup mbot_viz.launch.py
```
### Mapping
```bash
ros2 launch mbot_bringup mbot_bringup.launch.py 
```
Run the following command **on NoMachine**
```bash
ros2 launch mbot_navigation slam_toolbox_online_async_launch.py
```
- `slam_toolbox_online_async_launch.py` will also bring up the rviz showing the mapping process

Run teleop node to drive robot to explore the area
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


### Localization
TODO

### Navigation
TODO

## Individual packages
#### sllidar_ros2
ROS LiDAR driver - This will publish the topic `/scan`
```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py frame_id:=lidar_link 
```


#### mbot_navigation
- ROS package consists of slam_toolbox and nav2
- Please check the README file under /mbot_navigation for use details.

#### mbot_vision
- ROS package for camera interface, image rectification, and AprilTag detection
- Please check the README file under /mbot_vision for use details.

#### mbot_interfaces
- Where mbot custom messages and services are defined.
- The messages defined here are used on both Pi5 and Pico.

#### mbot_description
- ROS package of the URDF model