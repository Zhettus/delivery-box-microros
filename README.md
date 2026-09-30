# Delivery Box — ESP32-S3 firmware with micro-ROS

Firmware for the delivery box mounted on the quadruped robot, for the
**ESP32-S3-WROOM-1 N16R8** and the wiring in
*Delivery_Box_Wiring_N16R8 — Revision 4*.

This is `delivery-box-esp32s3` plus a **micro-ROS (ROS 2 Jazzy) node**. The robot
sets the box states over ROS 2 topics. The serial commands still work for
debugging. The micro-ROS setup (native USB transport, agent command) is the same
as in `hub-motor-microros`. See [ROS 2 interface](#ros-2-interface-micro-ros) below.

| Subsystem | Hardware | ESP32-S3 peripheral | Pins |
|---

## ROS 2 interface (micro-ROS)

### Two USB-C ports, two jobs

| Board port | Linux device | Used for |
|---|---|---|
| **"USB"** (native USB, GPIO 19/20) | `/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*` | **micro-ROS** ↔ agent |
| **"UART"/"COM"** (CH343 USB-UART bridge, GPIO 43/44) | `/dev/serial/by-id/usb-1a86_*` | flashing, logs, serial commands |

Connect **both** while developing. On the robot, only the "USB" port is needed.
Both ports show up as `/dev/ttyACM*` (the CH343 bridge uses the `cdc_acm` driver,
so there is **no** `/dev/ttyUSB0`), and whichever is plugged in first gets
`ttyACM0`. Use the `/dev/serial/by-id/` names above, or check with
`ls -l /dev/serial/by-id/`. Flashing also works through the native port, but the
logs and serial commands are only on the UART port.

### Run it

```bash
# 1. Flash (through the UART port) and watch the logs
. ~/.espressif/tools/activate_idf_v5.2.7.sh
cd ~/delivery-box-esp32s3-microros
idf.py -DMOTOR_TEST=0 -p /dev/serial/by-id/usb-1a86_* build flash monitor     # exit with Ctrl+]

# 2. Second terminal: start the agent on the native USB port (same as hub-motor)
source /opt/ros/jazzy/setup.bash && source ~/uros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200

# 3. Third terminal: talk to the box
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep delivery_box
```

The log shows `ros: connected: node 'delivery_box' is up`. If the agent is
restarted or the cable is re-plugged, the box reconnects on its own.

### Commands: topics the computer publishes

| Topic | Type | Values | What the box does |
|---|---|---|---|
| `/delivery_box/set_state` | `std_msgs/String` | a state name (below) | changes LEDs, eyes, tail light **and the door** together |
| `/delivery_box/door_cmd` | `std_msgs/String` | `open` | unlocks the latch, opens to 90° at 5 rpm and stays open |
| | | `close` | closes to 0° at 5 rpm, then locks the latch |
| | | `stop` | stops the door where it is (latch stays retracted if not closed) |
| | | `home` | closes slowly until the end stop and sets that as 0° (door position lost) |
| | | `unlock` | keeps the latch retracted even when closed (opening by hand, maintenance) |
| | | `lock` | back to automatic latch control (locks if the door is closed) |
| `/delivery_box/door_goto` | `std_msgs/Float32` | 0 … 90 | moves the door to that angle (e.g. `45.0` = half open), same speed and safety |

### Robot states (`set_state`)

| State | LED strips | Eyes | Tail light | Door |
|---|---|---|---|---|
| `startup` | white pulse | breathing | off | unchanged |
| `moving_with_task` | blue pulse | on | running light | **closes + locks** |
| `moving_without_task` | white pulse | on | running light | **closes + locks** |
| `success_waiting` | green pulse | on | running + brake | unchanged |
| `success_open_door` | solid green | on | running + brake | **unlocks + opens**, stays open |
| `success_close_door` | solid green | on | running + brake | **closes + locks** |
| `failure` | red pulse | on | running + brake + hazards | **stops where it is** |
| `failure_not_critical` | yellow pulse | on | running + brake | unchanged |
| `charging` | cyan comet around the box | breathing with the comet | off | unchanged |

- Setting the same state twice is harmless: the door goes **to** open/closed, never "by" an angle.
- A door that can't finish (resistance on every retry, timeout) switches the box to
  `failure_not_critical` **by itself**. This is the only state change the box makes on its own.
- An unknown state or door command is ignored and logged.

### Status: topics the box publishes

| Topic | Type | When | Content |
|---|---|---|---|
| `/delivery_box/state` | `std_msgs/String` | on change + every 1 s | current robot state |
| `/delivery_box/door_state` | `std_msgs/String` | on change + every 1 s | `closed` `opening` `open` `closing` `idle` `stopped` `backing_off` `retry_wait` `homing` or `fault:<reason>` |
| `/delivery_box/door_result` | `std_msgs/String` | when a move ends | `done` · `returned` (resistance every try, back at the start) · `blocked` · `stopped` · `timeout` · `sensor_fail` |
| `/delivery_box/latch` | `std_msgs/String` | on change + every 1 s | `locked` `unlocking` `unlocked` `locking` |
| `/delivery_box/door_angle` | `std_msgs/Float32` | 5 Hz | door angle in degrees, 0 = closed |
| `/delivery_box/power` | `sensor_msgs/BatteryState` | 1 Hz | INA228: motor supply voltage, motor current (negative = drawn), sensor temperature |

### Examples

```bash
# a delivery
ros2 topic pub --once /delivery_box/set_state std_msgs/msg/String "{data: moving_with_task}"
ros2 topic pub --once /delivery_box/set_state std_msgs/msg/String "{data: success_waiting}"
ros2 topic pub --once /delivery_box/set_state std_msgs/msg/String "{data: success_open_door}"
ros2 topic pub --once /delivery_box/set_state std_msgs/msg/String "{data: success_close_door}"
ros2 topic pub --once /delivery_box/set_state std_msgs/msg/String "{data: moving_without_task}"

# the door directly
ros2 topic pub --once /delivery_box/door_cmd  std_msgs/msg/String  "{data: open}"
ros2 topic pub --once /delivery_box/door_cmd  std_msgs/msg/String  "{data: close}"
ros2 topic pub --once /delivery_box/door_goto std_msgs/msg/Float32 "{data: 45.0}"

# watch
ros2 topic echo /delivery_box/state
ros2 topic echo /delivery_box/door_state
ros2 topic echo /delivery_box/door_result
ros2 topic echo /delivery_box/latch
```

`tools/ros_demo.sh` walks through a whole delivery with a pause between states.

### If the ROS connection is lost

- A moving door is **stopped** (`APP_ROS_STOP_DOOR_WHEN_LOST`).
- Set `APP_ROS_WARN_WHEN_LOST 1` to also switch to `failure_not_critical`.
- Everything else keeps its current look. The node reconnects by itself.

### micro-ROS build notes

- The micro-ROS library is built **inside this project** from
  `components/micro_ros_espidf_component` (copied from `hub-motor-microros`,
  which is left untouched). The first `idf.py build` takes a few minutes; later
  builds reuse `libmicroros.a`.
- `app-colcon.meta` sets the custom transport and the entity limits
  (6 publishers, 4 subscriptions). After changing it, run
  `idf.py clean-microros` once, then build again. That rebuilds the library.
- `sdkconfig.defaults` turns **off** the secondary console on the native USB port,
  because log text there would corrupt the micro-ROS stream.

---

## Quick start

```bash
# 1. Activate ESP-IDF 5.2.7 (installed with the EIM installer)
. ~/.espressif/tools/activate_idf_v5.2.7.sh

# 2. Build (the target esp32s3 is set by sdkconfig.defaults)
cd ~/delivery-box-esp32s3-microros
idf.py build

# 3. Flash through the "UART" USB-C port, then open the console there
idf.py -p /dev/serial/by-id/usb-1a86_* flash monitor        # exit the monitor with Ctrl+]
```

On boot you'll see every subsystem report `ready`, a summary line, and:

```
I (812) main: subsystems: door=OK tail=OK eyes=OK leds=OK i2c=OK power=OK mic=OK speaker=OK
EVT state startup
EVT ready (type 'help')
```

Anything shown as `--` failed to start (unplugged, wrong address…). The rest
keeps working. **First time on new hardware? Follow [docs/bring-up.md](docs/bring-up.md).**

Host unit tests (no board needed):

```bash
cd test/host && make        # door state machine, LED effects, WM8960 PLL, INA228 maths
```

---

## Robot states

The robot (for example a ROS node on the main computer) sets **one state** and
the box does the rest. Defined in [`main/robot_state.c`](main/robot_state.c):

| State | LED strips | Eyes | Tail light | Door | Sound |
|---|---|---|---|---|---|
| `startup` | white pulse | thinking (breathing) | off | — | start-up chime |
| `moving_with_task` | blue pulse | on | run | **close** | — |
| `moving_without_task` | white pulse | on | run | **close** | — |
| `success_waiting` | green pulse | on | run + brake | — | arrival chime |
| `success_open_door` | solid green | on | run + brake | **open** (stays open) | door chime |
| `success_close_door` | solid green | on | run + brake | **close** | door chime |
| `failure` | red pulse | on | run + brake + both indicators | **stop** | alarm |
| `failure_not_critical` | yellow pulse | on | run + brake | — | warning beeps |
| `charging` | cyan comet around the box | breathing in sync with the comet | off | — | — |

State sounds are currently **off** (`APP_AUDIO_STATE_SOUNDS 0` in `main/app_config.h`), so
the "Sound" column is what plays when you set it back to `1`. The speaker still works
for `beep` and your own code.

**Door in the states** (`STATES[]` in `main/robot_state.c`, values in `main/app_config.h`):

| State | Door action | How |
|---|---|---|
| `success_open_door` | **open** | `door_move_to(APP_DOOR_OPEN_DEG, APP_DOOR_OPEN_RPM, APP_DOOR_OPEN_LIMIT_A)` |
| `success_close_door`, `moving_with_task`, `moving_without_task` | **close** | `door_move_to(0, APP_DOOR_CLOSE_RPM, APP_DOOR_CLOSE_LIMIT_A)` |
| `failure` | **freeze** | `door_stop()` |
| all others | keep as it is | — |

- **Absolute moves:** states always go **to** open or closed (never "by" an angle),
  so sending a state twice is harmless.
- **Resistance:** each move goes back to where it started, waits and retries.
- **The robot never drives with the door open:** both *moving* states close it.
- **Door fault:** a door that gives up (`RETURNED`, `BLOCKED`, `TIMEOUT`) switches the box to
  `failure_not_critical` and sends `EVT door fault <reason> <result>`.

To change a look, edit that state's row in `STATES[]`. There is one row per state.

---

## Door behaviour

The door motor runs a closed-loop state machine at 50 Hz
([`components/door/door_fsm.c`](components/door/door_fsm.c), unit-tested):

```
 CLOSED ──open──► OPENING ──arrived──► OPEN ──close──► CLOSING ──arrived──► CLOSED
                     │                                    │
                interference                         interference
                     ▼                                    ▼
               BACKING_OFF  (go back to where the move started)
                     │ arrived
                     ▼
               RETRY_WAIT  (3 s) ──► try the same move again
                     │
          after 3 failed retries ──► FAULT (door is back at its start)
```

- **Interference** means the door keeps moving slower than 35 % of the commanded
  speed for 300 ms (the first 500 ms after starting or reversing are ignored).
- **Motor:** a brushless gear motor with its own built-in driver. The ESP32
  sets speed (PWM), direction (DIR) and brake, and counts the FG speed pulses
  for position. FG has no direction, so pulses are signed by the driven
  direction.
- **Before any reversal** the motor is stopped and braked, and the firmware
  waits until FG shows standstill. This is enforced in one place
  ([`motor_control.c`](components/door/motor_control.c)).
- A brushed motor + DRI0042 is still supported (`APP_DOOR_MOTOR_TYPE 1`).
- The door assumes it is **closed at power-up**. If it might not be, send
  `door home`: it closes slowly until it touches the end stop and zeroes there.
- `FAULT` clears with any new `door open` / `door close` / `door home`.

### Latch (solenoid lock)

Every door move is wrapped in the lock sequence
([`door_lock.c`](components/door/door_lock.c), unit-tested). Nothing else
has to handle it:

```
 LOCKED ──open──► UNLOCKING ──200 ms──► OPENING ──at 90°──► OPEN
 solenoid OFF      solenoid ON           solenoid ON         solenoid ON, motor off
 motor off         motor waits                                  │ `door close` or success_close_door
    ▲                                                           ▼
 LOCKED ◄──100 ms── LOCKING ◄──────at 0°, stopped─────────── CLOSING
                   motor off, solenoid ON                    solenoid ON
```

- The "door open / door closed sensors" are the motor's FG position (0° and
  `APP_DOOR_OPEN_DEG`). No limit switches are needed.
- The latch never extends while the door is away from 0°, so it stays retracted
  during resistance handling, retries and after a fault (the door can't latch
  there anyway). Moving again during LOCKING unlocks at once, with no wait.
- A command that doesn't move the door (e.g. `close` when already closed) doesn't
  touch the latch.
- Auto-close: `APP_DOOR_HOLD_OPEN_MS` (0 = stay open until `door close` / `success_close_door`; e.g. 30000 = close by itself after 30 s).
- Timings and hold power: `APP_LATCH_*` in `main/app_config.h`.
  `APP_LATCH_ENABLED 0` if no solenoid is fitted.

---

## Motor test

`main/motor_test.c` is a separate program for the door motor alone. It has
the same logic as the old ZK-BM1 program: PI speed control with
feed-forward, stall detection, and `set_motor_speed_rpm()` with a
+5 rpm / stop / −5 rpm loop. It is adapted to the brushless motor with its
own driver (SPEED PWM, DIR, BRAKE and FG pulses).

At start-up it runs a **wire check**. This finds which of blue (GPIO40),
green (GPIO18) and white (GPIO21) is SPEED, DIR and BRAKE, and at which
level. Paste the printed `#define` lines and set `WIRE_CHECK 0` to skip it.
Run it with the motor **not attached to the door**.

```bash
idf.py -DMOTOR_TEST=1 -p /dev/serial/by-id/usb-1a86_* build flash monitor   # motor test (output on the UART port)
idf.py -DMOTOR_TEST=0 -p /dev/serial/by-id/usb-1a86_* build flash monitor   # firmware with micro-ROS
```

---

## Moving the door from code

`components/door/include/door.h`:

```c
door_open_by(90.0f, 8.0f, 0.50f);    // open 1/4 turn at 8 rpm, current limit 0.5 A
door_close_by(120.0f, 8.0f, 0.50f);  // close 1/3 turn

door_move_t m = door_move_defaults(180.0f, 8.0f);   // half turn, full control:
m.current.limit_a = 0.40f;  m.current.rise_a = 0.08f;
m.return_rpm = 4.0f;  m.max_retries = 3;  m.retry_wait_ms = 3000;
door_result_t r = door_rotate(&m);   // DONE / RETURNED / BLOCKED / STOPPED / TIMEOUT / SENSOR_FAIL
```

- **Anchor:** every move remembers where it started. On resistance (motor current
  above the limit), the door stops, goes back to the anchor (counted in FG
  pulses, not seconds), waits, and retries.
- **Giving up:** after `max_retries` it stays at the anchor and returns `RETURNED`.
- **Blocked both ways:** if the way back is blocked too, it stops there and returns `BLOCKED`.
- **Timeout:** each attempt has a time limit (automatic: 2 × travel time + 3 s) as a
  backup to the pulse count.
- **Serial:** `door rotate <degrees> <rpm> [limit_A]` does the same from the serial port.
- **Bench test:** `main/door_motion_test.c` (settings at the top of the file).

---

## Serial commands

On the "UART"/"COM" USB-C port (`/dev/serial/by-id/usb-1a86_*`). The native "USB" port carries micro-ROS.

| Command | Example | What it does |
|---|---|---|
| `help` | | list commands |
| `status` | | one-line overview of everything |
| `states` | | list the robot states |
| `state [name]` | `state success_open_door` | get / set the robot state |
| `next` / `prev` | `next` | step to the next / previous state (list order) |
| `demo on [seconds] [door]` | `demo on 5 door` | cycle through a realistic delivery sequence; `door` = also move the door |
| `demo off` | | stop cycling |
| `door open\|close\|stop\|home\|status` | `door status` | door control + position, speed, duty, latch |
| `door unlock` / `door lock` | `door unlock` | keep the latch retracted even when closed (manual opening, solenoid test) / back to automatic |
| `leds <effect> [r g b] [time_s]` | `leds comet 0 255 255 2` | effects: off solid pulse blink comet fill diverge identify |
| `leds max <0-255>` | `leds max 120` | global LED brightness ceiling |
| `eyes off\|thinking\|constant [fade_s]` | `eyes thinking 1.5` | eye mood |
| `eyes cap <0-1>` | `eyes cap 0.2` | eye brightness ceiling |
| `tail brake\|run\|left\|right <0-255> [fade_ms]` | `tail brake 255` | one tail-light function |
| `tail off` | | all tail lights off |
| `volume [0-100]` | `volume 60` | speaker volume |
| `beep [freq_hz] [ms]` | `beep 880 300` | test tone |
| `mic [window_ms]` | `mic 1000` | microphone RMS / peak level in dBFS |
| `power [reset]` | `power` | INA228: voltage, current, power, energy |
| `i2c scan` | | should find 0x1A (WM8960) and 0x40 (INA228) |
| `reboot` | | restart |

Events: `EVT state <name>`, `EVT door <state> <fault> <result>`,
`EVT power low_voltage <V>`, `EVT power high_current <A>`, `EVT ready`.

Manual `leds` / `eyes` / `tail` commands override the current state's look
until the next `state …` command.

---

## Project layout

```
delivery-box-esp32s3-microros/
├── main/
│   ├── board.h            ← PIN MAP: single source of truth (+ compile-time pin checks)
│   ├── app_config.h       ← ALL tunables: brightness, door angles/speeds, volume, shunt…
│   ├── main.c             ← start-up order
│   ├── robot_state.c/.h   ← state table: what each robot state looks/sounds like
│   ├── demo.c/.h          ← cycles through all states (APP_DEMO_MODE / `demo on`)
│   ├── commands.c/.h      ← serial command interface
│   ├── door_setup.h       ← door config from board.h + app_config.h
│   ├── ros_interface.c/.h ← micro-ROS node: topics → states / door, status → topics
│   ├── usb_jtag_transport.c/.h ← micro-ROS transport over the native USB port
│   ├── motor_test.c       ← separate program: motor wire check + speed test
│   └── power_monitor.c/.h ← periodic INA228 readings + warnings
├── components/            ← one reusable driver per subsystem (no pins hard-coded)
│   ├── led_strips/        ← WS2815 ×4 over RMT; led_effects.c = pure pixel maths
│   ├── eyes/              ← JZ-MOS eyes, gamma + breathing
│   ├── tail_lights/       ← 4 dimmable channels with hardware fades
│   ├── door/              ← BLDC motor + FG, door_fsm, latch (door_lock), current guard, speed PI
│   ├── audio/             ← full-duplex I2S: tone player + microphone
│   ├── wm8960/            ← codec driver; wm8960_clock.c = PLL maths
│   ├── ina228/            ← power monitor; ina228_conv.c = unit conversions
│   └── pwm_util/          ← shared LEDC helpers
├── test/host/             ← PC unit tests for all hardware-independent logic
├── docs/
│   ├── wiring.md          ← wiring reference + hardware notes
│   ├── bring-up.md        ← first power-on checklist and calibration
│   └── migration.md       ← what changed from the old ESP32 code, and why
├── tools/ros_demo.sh    ← walks through a delivery over ROS 2
├── app-colcon.meta      ← micro-ROS transport + entity limits
├── sdkconfig.defaults     ← ESP32-S3, 16 MB flash, octal PSRAM, 1 kHz tick
└── CMakeLists.txt
```

**Where to change things:**

| I want to… | Edit |
|---|---|
| move a signal to another GPIO | `main/board.h` (the build refuses forbidden/duplicate pins) |
| change door angle, speed, retry timing | `main/app_config.h` → *Door* |
| change colours/sounds/door action of a state | `main/robot_state.c` → `STATES[]` |
| change LED count or the loop direction | `main/app_config.h` → *LED strips* |
| demo all states on the bench | `APP_DEMO_MODE 1` in `main/app_config.h` |
| add a serial command | `main/commands.c` → `COMMANDS[]` |

---

## Task / priority map

| Task | Priority | Period | Notes |
|---|---|---|---|
| `door` | 10 | 20 ms | highest: motor safety. Never prints or blocks; supervised by the task watchdog (2 s) |
| `door_evt` | 3 | on event | logs door events, runs the state callback |
| `audio` | 5 | on demand | tone synthesis → I2S |
| `eyes` | 4 | 10 ms | |
| `leds` | 3 | 10 ms | ~3 ms per frame for 96 pixels on 4 strips |
| `commands` | 2 | on input | |
| `power` | 1 | 1 s | |

---

## Open items (from the wiring document §9)

- **Mic streaming path (Wi-Fi → ROS topic)** is not decided yet. The I2S capture
  is ready: `audio_mic_read()` returns 32 kHz mono 24-bit samples. A streaming
  task only needs to call it in a loop and send the buffers.
- **Interference detection** uses speed deviation from the encoder (method
  "absolute threshold on speed ratio"). If a moving-average or baseline-profile
  method is preferred later, only `obstructed()` in `door_fsm.c` changes, and the
  host test `test_door_fsm.c` shows how to check it.
- **Acoustic sealing** for the mic (gasket, Gore IP67 vent) is unaffected by firmware.
# delivery-box-microros
