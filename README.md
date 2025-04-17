# mbot_ros2_ws
> mbot - ROS2 Jazzy - Ubuntu 24.04 - Gazebo Harmonic

Gazebo Harmonic is a new version of Gazebo and is next level annoying, most of the tutorials are outdated.

---

This is the workspace level, the `src` folder, which includes multiple packages.

### To get start:
```bash
mkdir mbot_ws
cd mbot_ws
git clone https://github.com/mbot-project/mbot_ros2_ws.git src
# or
gh repo clone mbot-project/mbot_ros2_ws src
```

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> ~/.bashrc
```

### Build 
```bash
cd ~/mbot_ws
colcon build --symlink-install
```
colcon does out of source builds. By default it will create the following directories as peers of the `src` directory:
- The `build` directory will be where intermediate files are stored. For each package a subfolder will be created in which e.g. CMake is being invoked.
- The `install` directory is where each package will be installed to. By default each package will be installed into a separate subdirectory.
- The `log` directory contains various logging information about each colcon invocation.

### Source the env
> In ROS2, it is recommended not to source the **workspace's** setup.bash file during a build. Namely not to outsource the job to `~/.bashrc`. [Source](https://robotics.stackexchange.com/questions/113595/ros2-recommended-handling-of-workspace-setup-bash)
```bash
source install/setup.bash
# below is the not recommended way
# echo "source $PWD/install/setup.bash" >> ~/.bashrc
```