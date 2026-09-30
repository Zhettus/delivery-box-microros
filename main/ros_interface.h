/*
 * ros_interface.h — micro-ROS node "delivery_box" (ROS 2 Jazzy).
 *
 * Transport: native USB Serial/JTAG (the USB-C port labelled "USB"), the
 * same custom transport as hub-motor-microros. On the PC:
 *
 *     ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
 *
 * Topics (all names start with /delivery_box/):
 *   subscribes  set_state   std_msgs/String   robot state, e.g. "success_open_door",
 *                                             "success_close_door"
 *               door_cmd    std_msgs/String   open | close | stop | home | unlock | lock
 *               door_goto   std_msgs/Float32  go to this angle (0 = closed ... 90 = open)
 *   publishes   state       std_msgs/String   current robot state      (on change + 1 Hz)
 *               door_state  std_msgs/String   e.g. "opening", "open", "fault:obstructed"
 *                                             (on change + 1 Hz)
 *               door_result std_msgs/String   how the last move ended: done | returned |
 *                                             blocked | stopped | timeout | sensor_fail
 *                                             (when a move ends)
 *               latch       std_msgs/String   locked | unlocking | unlocked | locking
 *                                             (on change + 1 Hz)
 *               door_angle  std_msgs/Float32  degrees, 0 = closed      (5 Hz)
 *               power       sensor_msgs/BatteryState  INA228: motor supply voltage
 *                                             and motor current        (1 Hz)
 *
 * The node reconnects on its own when the agent restarts or the cable is
 * re-plugged.
 */
#pragma once

#include "esp_err.h"

esp_err_t ros_interface_start(void);
