# mbot_navigation
This package provides **mapping**, **localization**, and **Nav2 navigation** for the MBot platform using `slam_toolbox` and `nav2_bringup`.

### Mapping
```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py frame_id:=lidar_link 
ros2 launch mbot_description rviz_launch.py
ros2 launch mbot_navigation slam_toolbox_online_async_launch.py
```
- Launch file: `mbot_navigation/launch/slam_toolbox_online_async_launch.py`
- Uses parameters from `mbot_navigation/config/slam_toolbox_online_async.yaml`
- Drive the robot until the map in RViz is complete.
    ```bash
    # Run teleop node to drive robot
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
### Save map
```bash
cd ~/mbot_ws/src/mbot_navigation/maps
ros2 run nav2_map_server map_saver_cli -f my_map_name
```
- Don't include any file extension.
- Always save it under `/maps` folder!!! Or you cannot use proviced launch files later.

### Visualize the map
```bash
ros2 launch mbot_navigation view_map.launch.py map_name:=your_map
```
- This launch file only does one thing, visualize your saved map.
- Remember do now include file extention in the command, just file name.

### Localization
```bash
TODO
```

### Navigation
```bash
TODO
```