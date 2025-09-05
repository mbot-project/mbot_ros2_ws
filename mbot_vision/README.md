# mbot_vision
ROS package for camera interface, image rectification, and AprilTag detection

This is for ROS Mbot use.

## Verify camera
To test if camera is detected, run:
```bash
rpicam-hello
```
- If the terminal didn't say `ERROR: *** no cameras available ***"`, your camera is detected.

To test if camera is working
```bash
rpicam-still -t 1 -o test.jpg
```
- This command will take a photo and save it in the current directory, the photo will be upside-down, that's expected.

To test if camera ros driver is working:
```bash
# bring up the camera node and flip the image
ros2 run camera_ros camera_node --ros-args \
-p orientation:=180 \
-p width:=640 -p height:=480 \
-p format:=BGR888
```

You can visualize it in rqt on NoMachine desktop:
```bash
ros2 run rqt_image_view rqt_image_view
```

## Camera Calibration
> If at any step it says package not found, source the `setup.bash`.

1. Find a checkerboard in the lab

    The size of the checkerboard is determined by the number of intersections, not squares. For instance, a notation of [8, 6] means the board has 8 intersections along one dimension and 6 along the other, corresponding to a grid of 9 by 7 squares. You need this information.
2. Run the camera node:
    ```bash
    ros2 run camera_ros camera_node --ros-args \
    -p orientation:=180 \
    -p width:=640 -p height:=480 \
    -p format:=BGR888
    ```
3. On NoMachine, run the following command, the GUI will pop up.
    ```bash
    ros2 run camera_calibration cameracalibrator \
    --size 6x8 --square 0.025 --ros-args -r image:=/camera/image_raw -p camera:=/camera
    ```
    - `--size` is the num of intersections we introduced at the step 1.
    - `--square` is the size of the block in meter. Here 0.025 means the block is 25 mm.
    - Adjust the values based on your checkerboard.
4. Use this [link](https://docs.nav2.org/tutorials/docs/camera_calibration.html#tutorial-steps) for reference if you don't know how to use the calibrator.
    - Read from: Tutorial Steps -> 5- Start the camera calibration node
5. Once you get the calibration result, move it to the mbot workspace:
    ```bash
    cp /tmp/calibrationdata.tar.gz ~/mbot_ws/src/mbot_vision
    cd ~/mbot_ws/src/mbot_vision
    tar -xvf calibrationdata.tar.gz -C calibration/
    ```
6. Rebuild the packages so the calibration file is accessible
    ```bash
    cd ~/mbot_ws
    colcon build
    ```

## Launch
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

