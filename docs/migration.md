# What changed from the old ESP32 project (`~/delivery-box`)

The old project is left untouched. This table maps every old file to its
replacement and says why it changed.

## File map

| Old (`delivery-box/main/…`) | New (`delivery-box-esp32s3/…`) | Notes |
|---|---|---|
| `final.c`, `Startup.c`, `Failure.c`, `Failure-not-critical.c`, `eyes_check.c`, `led-check.c` (six separate `app_main`s) | `main/main.c` + `main/commands.c` | One firmware. Each old test program is now a serial command (`state failure`, `eyes thinking`, `leds diverge`…) or demo mode. |
| `states.c/.h` | `main/robot_state.c/.h` | Same looks, now a table. Adds tail lights, sounds and door actions per state. |
| `components/led/led-pulsation.c`, `led-pulsation-4.c`, `led.c` | `components/led_strips/` | 4 strips on GPIO 4–7. One render task for every effect (the old code needed a second task and a "manual" flag for comet/diverge). |
| `components/eyes/eyes.c` | `components/eyes/` | Same breathing/gamma logic. Pins/channels injected; eyes had the **same LEDC channel 0 as the tail lights** before. |
| `components/tail-light/mosfet-tail-light.c` (+ `tail-light.c`, `relay.c`) | `components/tail_lights/` | P-MOSFET version only (the relay board is gone). 100 % duty is now true DC. |
| `components/door/gear-motor.c` | `components/door/` | ZK-BM1 → **DRI0042**, see below. |
| `components/door/dynamixel*.c` | — | The Dynamixel is not part of the new setup. |
| `components/microphone/ics43434.c` | `components/audio/` | Now shares I2S0 with the speaker in full duplex. The WM8960 is the clock master. Rate raised from 16 kHz to 32 kHz: the ICS43434 needs a BCLK of at least ~1.47 MHz (≈23 kHz × 64). |
| — | `components/wm8960/` | New: WM8960 speaker codec. |
| — | `components/ina228/`, `main/power_monitor.c` | New: INA228 current sensor. |

## Pin changes

| Signal | Old ESP32 | New ESP32-S3 |
|---|---|---|
| LED strips | 19, 18 (2 strips) | 4, 5, 6, 7 (4 strips) |
| Eyes | 19, 18 (clashed with LEDs!) / 23, 22 | 12, 13 |
| Tail brake/run/left/right | 25, 26, 33, 32 | 8, 9, 10, 11 |
| Motor | ZK-BM1 IN1/IN2 = 26, 27 | DRI0042 IN1 = 18, IN2 = 21, PWM = 40 |
| Encoder A/B | 32, 33 | 38, 39 |
| Mic BCLK/WS/DIN | 26, 25, 33 | 14, 15, 17 (shared clock) |
| Speaker (WM8960) | 21, 22, 26, 25, 27 (planned) | I2C 1/2, I2S 14/15/16 |

Several old pins (22–25, 26–32, 33–37) **do not exist or are used internally** on the
ESP32-S3 N16R8. The old code cannot simply be recompiled.

## Behaviour changes

### Door motor
- **Driver:** ZK-BM1 took PWM on both inputs at ≤ 2 kHz (audible whine).
  DRI0042 takes direction on IN1/IN2 and speed on a separate PWM pin, now at **20 kHz**.
- **Brake-before-reverse:** the DRI0042 datasheet requires ≥ 100 ms of STOP
  before changing direction. This is enforced by an interlock, so no code path can skip it.
- **Position control instead of timed moves:** the old `moveForward(seconds, rpm)`
  became real positions (`door open` goes to `APP_DOOR_OPEN_DEG`).
- **Interference handling** follows the project notes: interference → go back
  to the previous position → wait 3 s → retry. After 3 retries it reports a
  fault. The old stall check (in its code the counter was re-declared inside
  the loop, so it could never reach its threshold) is replaced by a
  speed-ratio detector that is unit-tested.
- The speed PI gains, feed-forward, minimum duty, encoder resolution and gear
  ratio are carried over from `gear-motor.c` (converted from 0..1023 to 0..1 duty).

### LEDs
- Same maths as before: perceptual dimming, colour correction `0xFFB0F0`,
  temporal dithering, R/G swap for the WS2815.
- The current limiter now reasons in **12 V milliamps**. The old one used 5 V
  WS2812 figures.
- The diverge effect mirrors about the true centre `(n-1)/2`. The old centre `n/2`
  was half a pixel off for odd strip lengths.
- RMT memory is set to 48 symbols per channel. The old value of 64 would use up two of the
  S3's four TX memory blocks per strip, so only two strips could run.

### Other
- The door control task never prints or waits on a lock. Events go through a
  queue to a low-priority task, the task watchdog supervises the loop, and a
  shutdown handler stops the motor before any software reboot.
- LEDC resources are allocated in one table (`board.h`), so no clashes are possible.
- Every subsystem starts independently. A missing HAT or sensor no longer stops the
  whole firmware (the old code used `ESP_ERROR_CHECK`, which reboots).
- The pin map is checked at compile time against the forbidden-pin list.
