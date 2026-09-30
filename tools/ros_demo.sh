#!/usr/bin/env bash
# Walk the delivery box through a whole delivery over ROS 2.
# Needs the micro-ROS agent running:
#   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
# Usage: tools/ros_demo.sh [seconds_per_state]   (default 8: the door needs
#        ~3 s to open or close at 5 rpm)
# Watch it in another terminal:  ros2 topic echo /delivery_box/door_state
set -euo pipefail
STEP=${1:-8}
source /opt/ros/jazzy/setup.bash

STATES=(startup moving_without_task moving_with_task success_waiting
        success_open_door success_close_door moving_without_task charging
        failure_not_critical failure startup)

for s in "${STATES[@]}"; do
    echo ">>> $s"
    ros2 topic pub --once /delivery_box/set_state std_msgs/msg/String "{data: $s}" >/dev/null
    sleep "$STEP"
done
echo "done"
