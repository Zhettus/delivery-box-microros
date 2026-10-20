/*
 * ros_interface.h — micro-ROS node "delivery_box" (ROS 2 Jazzy): the whole
 * box is controlled and reported over ROS topics.
 *
 * Transport: native USB Serial/JTAG (the USB-C port labelled "USB"), custom
 * XRCE-DDS transport (usb_jtag_transport.c). Logs go to the other port
 * ("UART"/"COM"). On the computer:
 *
 *     ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
 *
 * Topics (all start with ROS_TOPIC_PREFIX = /delivery_box):
 *
 *  the computer sends (box subscribes)
 *   set_state      std_msgs/String   a robot state (states.h), e.g. "success_open_door"
 *   door_cmd       std_msgs/String   open | close | stop | home | unlock | lock
 *   door_goto      std_msgs/Float32  go to this angle, 0 = closed .. DOOR_OPEN_ANGLE_DEG
 *   sound          std_msgs/String   startup | arrived | greeting | warning | failure |
 *                                    door_held | stop | volume <0-100>
 *   mic/enable     std_msgs/Bool     true = stream the microphone, false = stop
 *   speaker/audio  std_msgs/Int16MultiArray  (best effort) audio to play: mono int16 at
 *                                    SPEAKER_STREAM_RATE_HZ (16 kHz), 160 samples per
 *                                    message; layout.data_offset = index of the first
 *                                    sample (0 = a new stream). tools/speak.py sends it.
 *
 *  the box sends (box publishes)
 *   state          std_msgs/String   current robot state             (on change + 1 Hz)
 *   door_state     std_msgs/String   e.g. "opening", "open", "open:held" (went back after
 *                                    someone held the door), "fault:not_closed"
 *                                                                    (on change + 1 Hz)
 *   door_result    std_msgs/String   how the last move ended: done | returned (held) |
 *                                    blocked | stopped | timeout | sensor_fail | not_closed
 *   latch          std_msgs/String   locked | unlocking | unlocked | locking
 *                                                                    (on change + 1 Hz)
 *   door_angle     std_msgs/Float32  degrees, 0 = closed             (5 Hz, best effort)
 *   power          sensor_msgs/BatteryState  INA228: motor supply V + A (1 Hz, best effort)
 *   mic/status     std_msgs/Bool     microphone streaming?           (on change + 1 Hz)
 *   mic/audio      std_msgs/Int16MultiArray  (best effort) 160 samples = 10 ms of 16 kHz
 *                                    mono; layout.data_offset as above. tools/listen_live.py
 *   speaker/status std_msgs/String   "idle" or "playing", + buffered/lost/dropped/underruns
 *                                                                    (on change + 1 Hz)
 *
 * The node reconnects on its own when the agent restarts or the cable is
 * re-plugged. The microphone is OFF after every (re)connection.
 */
#pragma once

#include "esp_err.h"

esp_err_t ros_interface_start(void);
