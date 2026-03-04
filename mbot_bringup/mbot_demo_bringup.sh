#!/bin/bash
#
# Interactive MBot demo bringup script.
# Each command runs in its own tmux window.
#
# Usage: ./mbot_demo_bringup.sh
#
# Controls:
#   Ctrl+B then N       next window
#   Ctrl+B then P       previous window
#   Ctrl+B then W       list all windows
#   Ctrl+B then 0-4     jump to window by number
#   Ctrl+B then D       detach from tmux
#   Ctrl+B then C       create new terminal
#
# Shutdown:
#   tmux kill-session -t mbot_demo
#
# Re-attach (if disconnected):
#   tmux attach -t mbot_demo

SESSION="mbot_demo"

# Kill any existing session
tmux kill-session -t "$SESSION" 2>/dev/null

echo "=== MBot Demo Bringup ==="
echo ""

# --- Select SLAM source ---
echo "Select SLAM source:"
echo "  1) slam_toolbox"
echo "  2) mbot_slam_node"
read -rp "Enter choice [1/2]: " slam_choice

case "$slam_choice" in
    1) SLAM_SRC="slam_toolbox" ;;
    2) SLAM_SRC="mbot_slam_node" ;;
    *) echo "Invalid choice. Exiting."; exit 1 ;;
esac

# --- Select drive mode ---
echo ""
echo "Select drive mode:"
echo "  1) teleop"
echo "  2) click2drive"
read -rp "Enter choice [1/2]: " drive_choice

case "$drive_choice" in
    1) DRIVE_MODE="teleop" ;;
    2) DRIVE_MODE="click2drive" ;;
    *) echo "Invalid choice. Exiting."; exit 1 ;;
esac

echo ""
echo "  SLAM source: $SLAM_SRC"
echo "  Drive mode:  $DRIVE_MODE"
echo "========================="
echo "Launching in tmux session '$SESSION'..."
echo ""

# 1. Foxglove Bridge (creates the session)
tmux new-session -d -s "$SESSION" -n "foxglove" \
    "ros2 launch foxglove_bridge foxglove_bridge_launch.xml; exec bash"

# 2. MBot Bringup
tmux new-window -t "$SESSION" -n "bringup" \
    "ros2 launch mbot_bringup mbot_bringup.launch.py; exec bash"

# 3. SLAM
if [[ "$SLAM_SRC" == "slam_toolbox" ]]; then
    tmux new-window -t "$SESSION" -n "slam" \
        "ros2 launch mbot_navigation slam_toolbox_online_async_launch.py; exec bash"
else
    tmux new-window -t "$SESSION" -n "slam" \
        "source ~/mbot_ros_labs/install/setup.bash && ros2 run mbot_slam slam_node; exec bash"
fi

# 4. Drive mode
if [[ "$DRIVE_MODE" == "teleop" ]]; then
    tmux new-window -t "$SESSION" -n "teleop" \
        "ros2 run teleop_twist_keyboard teleop_twist_keyboard; exec bash"
    tmux select-window -t "$SESSION:teleop"
else
    tmux new-window -t "$SESSION" -n "motion_ctrl" \
        "source ~/mbot_ros_labs/install/setup.bash && ros2 run mbot_setpoint motion_controller_diff --ros-args -p use_localization:=true; exec bash"
    tmux new-window -t "$SESSION" -n "nav" \
        "source ~/mbot_ros_labs/install/setup.bash && ros2 run mbot_nav navigation_node --ros-args -p pose_source:=tf; exec bash"
fi

# Attach to session
tmux attach -t "$SESSION"
