# mbot_navigation
This package provides **mapping**, **localization**, and **Nav2 navigation** for the MBot platform using `slam_toolbox` and `nav2_bringup`.

## Mapping
```bash
ros2 launch mbot_bringup mbot_bringup.launch.py 
```
Run the following command **on NoMachine**
```bash
ros2 launch mbot_navigation slam_toolbox_online_async_launch.py
```
- `slam_toolbox_online_async_launch.py` will also bring up the rviz showing the mapping process
- Launch file: `mbot_navigation/launch/slam_toolbox_online_async_launch.py`
- Uses parameters from `mbot_navigation/config/slam_toolbox_online_async.yaml`

Run teleop node to drive robot to explore the area
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
## Save map
```bash
cd ~/mbot_ws/src/mbot_navigation/maps
ros2 run nav2_map_server map_saver_cli -f my_map_name
```
- Don't include any file extension.
- Always save it under `/maps` folder!!! Or you cannot use proviced launch files later.

## Visualize the map
```bash
ros2 launch mbot_navigation view_map.launch.py map_name:=your_map
```
- This launch file only does one thing, visualize your saved map.
- Remember do not include file extention in the command, just file name.
- Your map has to be under `/maps` folder.

## Localization
```bash
ros2 launch mbot_bringup mbot_bringup.launch.py 
```
Run the following command **on NoMachine**
```bash
ros2 launch mbot_navigation localization_launch.py map_name:=your_map_name
```
Then on Rviz:
1. Set the Initial Pose. Click the "2D Pose Estimate" button in the RViz toolbar. On the map, click and drag an arrow to represent where your robot is in the real world and which way it's facing.
2. Release the mouse button. You should see the Lidar scans (the red dots) "snap" into place, aligning with the map walls. AMCL's particle cloud should converge around the robot's true location.
3. Test It! Manually drive your robot around. As you do, watch its position in RViz. It should move in perfect sync with the real robot, with its Lidar scans staying aligned with the map's walls.
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
### Navigation
```bash
TODO
```