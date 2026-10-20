# Delivery Box — ESP32-S3 firmware, controlled over micro-ROS

Firmware for the delivery box on the **ESP32-S3-WROOM-1 N16R8**, with the same
hardware setup and logic as `delivery-box-final`, and **everything controlled
and reported over ROS 2 (Jazzy) topics** through micro-ROS:

- robot states (LED strip, face, tail lights, sound, door together)
- the door: open / close / go to an angle / stop / home / latch, with
  **door-held detection** (INA228 motor current: closing → opens again,
  opening → closes again)
- status: state, door state + result, latch, door angle, motor supply power
- **microphone → computer**: the ICS43434, streamed live (16 kHz)
- **computer → speaker**: audio sent in 10 ms pieces and played on the WM8960
  HAT (`tools/speak.py`: a WAV file, the computer's microphone, or a tone)
- built-in sounds and speaker volume

## Two USB-C ports, two jobs

| Board port | Linux device | Used for |
|---|---|---|
| **"USB"** (native USB, GPIO 19/20) | `/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*` | **micro-ROS** ↔ agent (flashing works too) |
| **"UART"/"COM"** (CH343 USB-UART bridge, GPIO 43/44) | `/dev/serial/by-id/usb-1a86_*` | logs (`idf.py monitor`), flashing |

Both ports show up as `/dev/ttyACM*` on Linux (whichever is plugged in first
gets `ttyACM0`), so use the `/dev/serial/by-id/` names. On a Mac:
`/dev/cu.usbmodem*` is the native port, `/dev/cu.usbserial*` /
`/dev/cu.wchusbserial*` the UART one. **Never open a monitor on the native port**
while ROS runs: one program per port.

## Run it

```bash
# 1. Build + flash through the UART port and watch the logs
. ~/.espressif/tools/activate_idf_v5.2.7.sh
cd ~/delivery-box-esp32s3-microros
idf.py -p /dev/serial/by-id/usb-1a86_* build flash monitor          # Ctrl+] quits

# 2. Second terminal: the agent on the native USB port
source /opt/ros/jazzy/setup.bash && source ~/uros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial \
     --dev /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_* -b 115200

# 3. Third terminal: talk to the box
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep delivery_box
```

The log shows `ros: connected: node 'delivery_box' is up`. If the agent restarts
or the cable is re-plugged, the box reconnects by itself.

**Power up with the door closed:** the shaft position at power-up is "closed" (0°).

## Topics

All names start with `/delivery_box/` (`ROS_TOPIC_PREFIX` in `main/config.h`).

### The computer sends

| Topic | Type | Values | What the box does |
|---|---|---|---|
| `set_state` | `std_msgs/String` | a state name (below) | LEDs, face, tail lights, sound **and the door** together |
| `door_cmd` | `std_msgs/String` | `open` | unlocks the latch, nudges, opens to `DOOR_OPEN_ANGLE_DEG` |
| | | `close` | closes until the door switch, seats, latch locks |
| | | `stop` | stops where it is |
| | | `home` | closes slowly to the end stop and takes that as 0° |
| | | `unlock` / `lock` | keep the latch retracted (open by hand) / automatic again |
| `door_goto` | `std_msgs/Float32` | 0 … 90 | goes to that angle (0 = closed) |
| `sound` | `std_msgs/String` | `startup` `arrived` `greeting` `warning` `failure` `door_held` | plays that sound |
| | | `stop` | stops the sound and the speaker stream |
| | | `volume 70` | speaker volume 0 … 100 |
| `mic/enable` | `std_msgs/Bool` | `true` / `false` | streams the microphone / stops |
| `speaker/audio` | `std_msgs/Int16MultiArray` | 160 samples, 16 kHz mono | plays it (best effort, see below) |

### The box sends

| Topic | Type | When | Content |
|---|---|---|---|
| `state` | `std_msgs/String` | change + 1 Hz | current robot state |
| `door_state` | `std_msgs/String` | change + 1 Hz | `closed`, `opening`, `open`, `closing`, …; `open:held` / `closed:held` after the door was held and went back; `fault:not_closed`, `fault:blocked`, … |
| `door_result` | `std_msgs/String` | when a move ends | `done`, `returned` (held), `blocked`, `stopped`, `timeout`, `sensor_fail`, `not_closed` |
| `latch` | `std_msgs/String` | change + 1 Hz | `locked`, `unlocking`, `unlocked`, `locking` |
| `door_angle` | `std_msgs/Float32` | 5 Hz | degrees, 0 = closed |
| `power` | `sensor_msgs/BatteryState` | 1 Hz | motor supply voltage and current (INA228) |
| `mic/status` | `std_msgs/Bool` | change + 1 Hz | microphone streaming? |
| `mic/audio` | `std_msgs/Int16MultiArray` | 100 Hz while on | 160 samples = 10 ms of 16 kHz mono |
| `speaker/status` | `std_msgs/String` | change + 1 Hz | `idle`/`playing buffered=…ms lost=… dropped=… underruns=…` |

### Examples

```bash
ros2 topic pub --once /delivery_box/set_state std_msgs/msg/String "{data: success_open_door}"
ros2 topic pub --once /delivery_box/door_cmd  std_msgs/msg/String "{data: close}"
ros2 topic pub --once /delivery_box/door_goto std_msgs/msg/Float32 "{data: 45.0}"
ros2 topic pub --once /delivery_box/sound     std_msgs/msg/String "{data: 'volume 60'}"
ros2 topic echo /delivery_box/door_state
tools/ros_demo.sh                 # a whole delivery, state by state
```

## Microphone → computer

```bash
python3 tools/listen_live.py              # switches it on, plays it, off on Ctrl+C
python3 tools/listen_live.py --wav a.wav  # also save it;  --no-play: statistics only
```

Pipeline on the board: I2S 48 kHz → high-pass, automatic gain, limiter →
16 kHz → 10 ms chunks → `mic/audio`. `layout.data_offset` is the index of the
first sample: the listener fills lost chunks with silence. The mic is OFF after
boot and after every reconnect. Tuning: `MIC_*` in `main/config.h`.

## Computer → speaker ("audio by bits")

The mirror image of the microphone: the computer cuts the audio into 10 ms
pieces (160 samples, 16 kHz mono int16), numbers them by sample
(`layout.data_offset`, 0 = a new stream) and sends them in real time on
`speaker/audio`. The box:

1. puts each piece in a 500 ms buffer; a gap in the numbers (lost piece) is
   filled with silence, so the timing stays right; old/duplicate pieces are
   dropped,
2. starts playing once 100 ms are buffered (absorbs USB/ROS jitter),
3. upsamples 16 → 48 kHz (`components/audio/audio_resample.c`) and plays it,
4. stops 300 ms after the last piece.

```bash
python3 tools/speak.py --wav hello.wav           # any PCM WAV: converted to 16 kHz mono
python3 tools/speak.py --live                    # talk into the computer (arecord)
python3 tools/speak.py --tone 440 --seconds 2
python3 tools/speak.py --wav hello.wav --volume 70 --gain 1.5
ros2 topic echo /delivery_box/speaker/status     # lost / dropped / underruns
```

A built-in sound (a state change, `sound`) plays in between and then the
stream continues. Settings: `SPEAKER_*` in `main/config.h`.

## Robot states (`set_state`)

| State | Strip | Face | Tail | Door | Sound |
|---|---|---|---|---|---|
| `startup` | white pulse | thinking | off | – | start-up notes |
| `moving_with_task` | blue pulse | on | off | closes | – |
| `moving_without_task` | white pulse | on | off | closes | – |
| `success_waiting` | green pulse | on | off | – | two notes |
| `success_open_door` | green solid | on | off | opens | `gm.wav` |
| `success_close_door` | green solid | on | off | closes + locks | – |
| `failure` | red pulse | on | brake | stops | falling notes |
| `failure_not_critical` | yellow pulse | on | off | – | two beeps |
| `charging` | cyan comet | follows the comet | off | – | – |

If the door gives up (stuck both ways, timed out, not closed), the box switches
to `failure_not_critical` by itself. `DEMO_MODE 1` cycles through the states
until the first ROS command.

## Someone holds the door

While the door moves, the INA228 reads the motor current every 5 ms. Held =
current above `DOOR_CURRENT_LIMIT_A`, or `DOOR_CURRENT_RISE_A` above the normal
running current, for `DOOR_CURRENT_CONFIRM_MS` (also: door much slower than
commanded). Then:

| Door was | Held → it | Ends as |
|---|---|---|
| closing | stops, opens again | `open:held`, result `returned` |
| opening | stops, closes again (seats on the switch, latch locks) | `closed:held`, result `returned` |
| going back, held again | stops where it is | `fault:blocked` |

The box beeps (`door_held` sound). The thresholds are estimates **[MEASURE]**:
measure with `motor-audio-check` (`idf.py -DCURRENT_LOG=1`).

## Pins

Same as `delivery-box-final` (`main/board.h`, checked at compile time):

| GPIO | Signal |
|---|---|
| 1, 2 | I2C SDA / SCL, 100 kHz: INA228 (0x40) + WM8960 HAT (0x1A) |
| 4 | LED strip data (one WS2815 strip, 134 LEDs) |
| 5 | door-closed switch to GND (closed = LOW) |
| 8, 9, 10, 11 | tail light brake / run / left / right |
| 12 | face: eyes + mouth (one XY-MOS) |
| 14, 15, 16 | I2S BCLK / WS / DOUT → WM8960 P12 / P35 / P40 (+ ICS43434 BCLK / LRCL) |
| 17 | I2S DIN ← ICS43434 DOUT (SEL → GND, 3V → 3.3 V) |
| 18 / 21 / 40 / 38 | motor PWM / DIR / BRAKE / FG |
| 41 | door latch solenoid (XY-MOS) |
| 19, 20 | native USB: micro-ROS |
| 43, 44 | UART0: logs |

## Building

### micro-ROS library

`components/micro_ros_espidf_component` (git submodule: branch `jazzy`, commit
`16cf0d5`, plus the local `PATH` line in `libmicroros.mk`; for a new computer
see `MICRO_ROS_NEW_COMPUTER_SETUP.md`, in ~/Downloads). `app-colcon.meta` sets the custom transport
and the limits: **10 publishers, 6 subscriptions, history 8** (the node uses 9
and 6).

After changing `app-colcon.meta`, rebuild only the compiled part, keeping the
pinned sources in `micro_ros_src/src` (`idf.py clean-microros` would delete
them and download again):

```bash
C=components/micro_ros_espidf_component
rm -rf $C/libmicroros.a $C/include $C/micro_ros_src/build $C/micro_ros_src/install $C/micro_ros_src/log
idf.py build                       # rebuilds the library: several minutes
```

`sdkconfig.defaults` puts the console on UART0 and turns the secondary console
off: log text on the native port would corrupt the micro-ROS stream.

### Tests on the computer

```bash
cd test/host && make      # door state machine + mic DSP + speaker upsampler
```

## Project layout

| Path | What |
|---|---|
| `main/main.c` | starts everything, then ROS is in control |
| `main/ros_interface.c` | the micro-ROS node: all topics |
| `main/states.c` | robot states (own task: setting one never blocks ROS) |
| `main/motor.c` | door front end (open/close with held-door reversal, go-to, home, latch, supply) |
| `main/mic_stream.c` | microphone capture → 16 kHz chunks |
| `main/sound.c` | built-in sounds + speaker stream settings |
| `main/led.c`, `face.c`, `tail_light.c` | LED strip, face, tail lights |
| `main/demo.c` | `DEMO_MODE` |
| `main/usb_jtag_transport.c` | micro-ROS transport over the native USB port |
| `main/config.h` / `board.h` | every tunable value / every pin |
| `components/door` | door control: speed loop, latch, switch, current guard, held → go back |
| `components/audio` | one full-duplex I2S: WM8960 playback, ICS43434 capture, `mic_dsp`, `audio_resample`, speaker stream buffer |
| `components/ina228` | current sensor driver |
| `tools/listen_live.py` | listen to the box microphone |
| `tools/speak.py` | send audio to the box speaker |
| `tools/ros_demo.sh` | walk through a delivery over ROS |

## Changes from the previous version (branch `main`)

- Hardware setup of `delivery-box-final`: one LED strip (134), one face output,
  door-closed switch on GPIO 5, tail lights on/off, I2C at 100 kHz, WM8960 set up
  as in `motor-audio-check` (ESP32 = I2S master, 48 kHz).
- Door logic of `delivery-box-final`: switch seating, latch timing, open nudge,
  held door → back to the previous state (no automatic retries by default).
- New topics: `sound`, `mic/enable`, `mic/status`, `mic/audio`, `speaker/audio`,
  `speaker/status`; `door_state` shows `:held`.
- The serial command shell, the motor test program and the old docs are gone
  (ROS is the control path); they are in the git history.

## Not yet checked on the hardware

- Speaker with 32-bit I2S slots (needed by the microphone): it should play as
  with 16-bit slots in `motor-audio-check`.
- Microphone at 48 kHz (the hand-off tested 32 kHz).
- Streaming both ways at once over one USB link (~35 kB/s each way).
- If the audio HAT's supply runs through the INA228 shunt, speaker current
  during a door move could look like a held door.
