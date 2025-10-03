# mbot_navigation
This package provides **mapping**, **localization**, and **Nav2 navigation** for the MBot platform using `slam_toolbox` and `nav2_bringup`.

**IMPORTANT!!!** - We marked how to use the terminals, it is not mandatory, but for the best performance, **only use NoMachine for visualiztion, nothing else.**

## Teleop mapping
**VSCode Terminal #1**: launch the robot model, TF, LiDAR node.
```bash
ros2 launch mbot_bringup mbot_bringup.launch.py 
```

**VSCode Terminal #2**: launch slam_toolbox
```bash
ros2 launch mbot_navigation slam_toolbox_online_async_launch.py
```
- Launch file: `mbot_navigation/launch/slam_toolbox_online_async_launch.py`
- Uses parameters from `mbot_navigation/config/slam_toolbox_online_async.yaml`


**NoMachine Terminal #1**: run Rviz
```bash
cd ~/mbot_ws/src/mbot_navigation/rviz
ros2 run rviz2 rviz2 -d mapping.rviz
```

**VSCode Terminal #3**: Run teleop node to drive robot to explore the area
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Save map
**VSCode Terminal #4**: Once you are done with mapping the entire area, to save map
```bash
cd ~/mbot_ws/src/mbot_navigation/maps
ros2 run nav2_map_server map_saver_cli -f my_map_name
```
- Don't include any file extension.
- Always save it under `/maps` folder!!! Or you cannot use proviced launch files later.

**VSCode Terminal #4**
```bash
cd ~/mbot_ws
colcon build
source install/setup.bash 
```
- Remember to compile the package so the map become available anywhere.


## Exploration
**VSCode Terminal #1**: launch the robot model, TF, LiDAR node.
```bash
ros2 launch mbot_bringup mbot_bringup.launch.py 
```

**NoMachine Terminal #1**: run Rviz
```bash
cd ~/mbot_ws/src/mbot_navigation/rviz
ros2 run rviz2 rviz2 -d mapping.rviz
```

**VSCode Terminal #2**: launch exploration node
```bash
ros2 launch mbot_navigation slam_exploration_launch.py
```
- In the launch file `slam_exploration_launch.py`, we can switch between brute force search for frontier `bruteforce_frontier_explorer.py`, and wavefront algorithm `wavefront_frontier_explorer.py`.
- The robot will start moving once the tf and map is available.


### Save map
**VSCode Terminal #3**: Once you are done with mapping the entire area, to save map
```bash
cd ~/mbot_ws/src/mbot_navigation/maps
ros2 run nav2_map_server map_saver_cli -f my_map_name
```
- Don't include any file extension.
- Always save it under `/maps` folder!!! Or you cannot use proviced launch files later.

**VSCode Terminal #3**
```bash
cd ~/mbot_ws
colcon build
source install/setup.bash 
```
- Remember to compile the package so the map become available anywhere.

## Visualize the map
**VSCode Terminal #1**: launch the map server
```bash
ros2 launch mbot_navigation view_map.launch.py map_name:=your_map
```
- Remember do not include file extention in the command, just file name.
- Your map has to be under `/maps` folder.

**NoMachine Terminal #1**: run Rviz
```bash
cd ~/mbot_ws/src/mbot_navigation/rviz
ros2 run rviz2 rviz2 -d view_map.rviz
```

## Localization
**VSCode Terminal #1**: launch the robot model, TF, LiDAR node.
```bash
ros2 launch mbot_bringup mbot_bringup.launch.py 
```
**VSCode Terminal #2**: launch localization node
```bash
ros2 launch mbot_navigation localization_launch.py map_name:=your_map_name
```

**NoMachine Terminal #1**: run Rviz
```bash
cd ~/mbot_ws/src/mbot_navigation/rviz
ros2 run rviz2 rviz2 -d localization.rviz
```
- Once the rviz show up, you will find that the robot model is blank, and there are errors. That is because there is no map -> odom -> base_footprint transform relation yet. Follow the steps below.

On Rviz:
1. Set the Initial Pose. Click the "2D Pose Estimate" button in the RViz toolbar. On the map, click and drag an arrow to represent where your robot is in the real world and which way it's facing.
2. Release the mouse button. You should see the Lidar scans (the red dots) "snap" into place, aligning with the map walls. You don't have to keep adjusting it perfectly. One good initial guess is enough. Use the teleop node to rotate the robot in place a little bit, or drive it forward and back a short distance. AMCL's particle cloud should converge around the robot's true location.
3. Test It! Manually drive your robot around. As you do, watch its position in RViz. It should move in perfect sync with the real robot, with its Lidar scans staying aligned with the map's walls.

    **VSCode Terminal #3**: Run teleop node to drive robot to explore the area
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

## Navigation
**VSCode Terminal #1**: launch the robot model, TF, LiDAR node.
```bash
ros2 launch mbot_bringup mbot_bringup.launch.py 
```
**VSCode Terminal #2**: launch navigation node
```bash
ros2 launch mbot_navigation navigation_launch.py map_name:=your_map_name
```

**NoMachine Terminal #1**
```bash
cd ~/mbot_ws/src/mbot_navigation/rviz
ros2 run rviz2 rviz2 -d navigation.rviz 
```


Once RViz appears, you must first localize the robot.
- Set the initial pose using the "2D Pose Estimate" button.
- Drive the robot around a little with the `teleop_twist_keyboard` to ensure the laser scan is well-aligned with the map and the pose is stable.

    **VSCode Terminal #3**: Run teleop node to drive robot to help robot localize itself
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

After the robot is localized, you can give it a navigation goal.
- Click the "2D Goal Pose" button in the RViz toolbar.
- Click on a destination on the map and drag an arrow to specify the desired final orientation.
- When you release the mouse button, the robot will begin planning a path and navigating to the goal, avoiding obstacles along the way.
