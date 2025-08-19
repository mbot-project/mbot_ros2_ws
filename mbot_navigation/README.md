# mbot_navigation
This package provides **mapping**, **localization**, and **Nav2 navigation** for the MBot platform using `slam_toolbox` and `nav2_bringup`.

### Mapping
```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py frame_id:=lidar_link 
ros2 launch mbot_description rviz_launch.py
ros2 launch mbot_navigation online_async_launch.py
```
- Launch file: `mbot_navigation/launch/online_async_launch.py`
- Uses parameters from `mbot_navigation/config/mapper_params_online_async.yaml`
- Drive the robot until the map in RViz is complete.
```bash
# Run teleop node to drive robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### Save map
- Method 1: Using the RViz Plugin: Go to Panels → Add New Panel → SlamToolbox Plugin
- Method 2: Nav2 map saver 
    ```bash
    cd ~/mbot_ws/src/mbot_nagivation/maps
    ros2 run nav2_map_server map_saver_cli -f my_map_name
    ```
    - Don't include any file extension.
  
### Visualize the map
```bash
# Terminal 1: Start map server with your map
ros2 launch nav2_bringup bringup_launch.py map:=/path/to/your/map.yaml

# Terminal 2: Open RViz with map frame
rviz2
```
Then in RViz:
- Change Fixed Frame to map
- Add Map display from /map topic

### Localization
```bash
ros2 launch mbot_navigation slam_toolbox_localization.launch.py
```

### Navigation
```bash
ros2 launch mbot_navigation nav2_navigation.launch.py \
    map:=/path/to/your/map.yaml
```