# mbot_ros2_ws
> mbot - ROS2 Jazzy - Ubuntu 24.04 - Gazebo Harmonic

This is the workspace level, the `src` folder, which includes multiple packages.

### To get started:
```bash
mkdir mbot_ws
cd mbot_ws
git clone --recurse-submodules https://github.com/mbot-project/mbot_ros2_ws.git src
```
or for `gh` users:
```bash
gh repo clone mbot-project/mbot_ros2_ws src
git submodule update --init --recursive
```
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> ~/.bashrc
```

### Build 
```bash
source /opt/ros/jazzy/setup.bash
cd ~/mbot_ws
colcon build --symlink-install
echo "source $PWD/install/local_setup.bash" >> ~/.bashrc
```
colcon does out of source builds. By default it will create the following directories as peers of the `src` directory:
- The `build` directory will be where intermediate files are stored. For each package a subfolder will be created in which e.g. CMake is being invoked.
- The `install` directory is where each package will be installed to. By default each package will be installed into a separate subdirectory.
- The `log` directory contains various logging information about each colcon invocation.