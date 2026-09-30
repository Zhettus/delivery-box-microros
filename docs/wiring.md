# Wiring reference — ESP32-S3-WROOM-1 N16R8 (Revision 4)

A condensed copy of *Delivery_Box_Wiring_N16R8 — Revision 4*, plus notes the
firmware depends on. The pin numbers are also in `main/board.h`, and the build
checks them.

## Master pin map

| GPIO | Signal | Goes to |
|---|---|---|
| 1 | I2C_SDA | WM8960 P3 + INA228 SDA (4.7 kΩ pull-up to 3.3 V) |
| 2 | I2C_SCL | WM8960 P5 + INA228 SCL (4.7 kΩ pull-up to 3.3 V) |
| 4 | LED_DATA_1 | WS2815 strip 1 DI (+ BI) |
| 5 | LED_DATA_2 | WS2815 strip 2 DI (+ BI) |
| 6 | LED_DATA_3 | WS2815 strip 3 DI (+ BI) |
| 7 | LED_DATA_4 | WS2815 strip 4 DI (+ BI) |
| 8 | TAIL_BRAKE | 1 kΩ → 2N7000 gate (brake channel) |
| 9 | TAIL_RUN | 1 kΩ → 2N7000 gate (running channel) |
| 10 | TAIL_LEFT | 1 kΩ → 2N7000 gate (left indicator) |
| 11 | TAIL_RIGHT | 1 kΩ → 2N7000 gate (right indicator) |
| 12 | EYE_L | JZ-MOS TRIG/PWM (left eye) |
| 13 | EYE_R | JZ-MOS TRIG/PWM (right eye) |
| 14 | I2S_BCLK | WM8960 P12 + ICS43434 BCLK |
| 15 | I2S_WS | WM8960 P35 (LRCLK) + ICS43434 LRCL |
| 16 | I2S_DOUT | WM8960 P40 (DACDAT) |
| 17 | I2S_DIN | ICS43434 DOUT |
| 18 | MOTOR_PWM | motor **green** wire: speed (PWM, HIGH = fast) |
| 21 | MOTOR_DIR | motor **white** wire: direction (CW/CCW) |
| 40 | MOTOR_BRAKE | motor **blue** wire: brake (HIGH = brake) |
| 38 | MOTOR_FG | motor **yellow** wire: FG speed pulses |
| 39 | (spare) | only used as Hall B if a brushed motor + DRI0042 is fitted |
| 41 | DOOR_LATCH | XY-MOS **TRIG/PWM** → latch solenoid (HIGH = unlocked) |

Spare: **42, 47, 48**.

**Never use:** 0, 3, 45, 46 (strapping) · 19, 20 (native USB) · 22–25 (don't exist)
· 26–32 (flash) · 33–37 (octal PSRAM) · 43, 44 (UART0 console/flashing).

## Subsystems

### Tail light (×4 identical channels)
```
+12V ──┬──────────────── P-FET Source
       └─ 10 kΩ ─┬────── P-FET Gate            P-FET Drain ── lamp function wire
                 └────── 2N7000 Drain
GPIO ── 1 kΩ ─────────── 2N7000 Gate          2N7000 Source ── GND
```
GPIO **HIGH = lamp ON**. Firmware PWM: 500 Hz, level 255 = 100 % (steady DC).

### LED strips WS2815 ×4
GND → common GND · DI → its GPIO · BI → same GPIO (or GND) · +12 V → 12 V supply.
Firmware assumes **RGB byte order** and pixel counts 29 / 29 / 19 / 19
(`app_config.h`).

### Eyes (JZ-MOS ×2)
VIN± → supply, VOUT± → eye LED, GND wire → common GND, TRIG/PWM → GPIO 12 / 13.
Firmware PWM: 1.5 kHz, gamma 2.2, pulses shorter than 20 µs are skipped, and brightness is capped at 50 % (`APP_EYES_BRIGHTNESS_CAP`).

### WM8960 Audio HAT
P1/P17 → 3.3 V · P3 → GPIO1 · P5 → GPIO2 · P12 → GPIO14 · P35 → GPIO15 ·
P40 → GPIO16 · P2/P4 → **dedicated 5 V buck** (not the dev board 5 V, not DRI0042 5VO)
· GND → common GND.
There is no MCLK wire: the codec runs from the HAT's own crystal through its PLL
and is the **I2S clock master**. It drives BCLK (GPIO14) and LRCLK (GPIO15);
the ESP32 and the ICS43434 follow them. Set the crystal frequency in
`APP_AUDIO_CODEC_MCLK_HZ`. If the HAT is missing, the ESP32 drives the clocks
itself so the microphone still works.

### ICS43434 microphone
SEL → GND (left slot) · LRCL → GPIO15 · BCLK → GPIO14 · DOUT → GPIO17 · 3V → 3.3 V · GND.

### Door motor: brushless (BLDC) gear motor with built-in driver

The motor has its own driver inside. **Red/black are its power input, not
the coils.** No external motor driver is used. The ESP32 only talks to the
control wires.

```
supply +12 V ──► INA228 Current+ ──[R002]──► Current− ──► motor RED  (+)
supply GND  ◄──────────── STAR POINT ◄───────────────────── motor BLACK (−)
                              ▲  ▲  ▲
                              │  │  └── ESP32 GND   (control signals need it)
                              │  └───── INA228 GND (and V−)
                              └──────── (all other grounds)

ESP32 GPIO18 ──► motor GREEN (speed PWM, HIGH = fast)
ESP32 GPIO21 ──► motor WHITE (direction)
ESP32 GPIO40 ──► motor BLUE  (brake, HIGH = brake)
ESP32 GPIO38 ◄── motor FG    (yellow, speed pulses) + 10 kΩ pull-up to 3.3 V
```

| Motor wire | Connect to | Note |
|---|---|---|
| **red** (+) | INA228 **Current−** | the supply + reaches the motor only **through** the INA228 |
| **black** (−) | supply GND (star point) | |
| **green** (PWM, speed) | GPIO18 | found by the wire check in `main/motor_test.c` |
| **white** (DIR, CW/CCW) | GPIO21 | |
| **blue** (BRAKE) | GPIO40 | HIGH = brake on |
| **yellow** (FG) | GPIO38 + **10 kΩ to 3.3 V** | FG is usually open-collector |

**Check the logic voltage of the control wires first.** With the motor
powered and PWM/DIR/BRAKE/FG **unconnected**, measure each against black:
- about **0 V or ≤ 3.3 V**: connect directly to the ESP32 as above.
- about **5 V**: the motor pulls its inputs up to 5 V, and the ESP32 pins are
  **not** 5 V tolerant. Put a 4-channel logic-level shifter (BSS138 module,
  LV = 3.3 V, HV = 5 V) between the ESP32 and PWM, DIR, BRAKE and FG.

Never swap red and black, and never change DIR while the motor spins. The
firmware stops and waits for standstill (no FG pulses) before reversing.

Unknown values (PWM polarity, which DIR level opens, brake polarity, FG
pulses per turn) are found with the identify test: `docs/bring-up.md`, step 5.

### Door motor, alternative: brushed motor + DRI0042
Set `APP_DOOR_MOTOR_TYPE 1`. The same pins then mean: GPIO21 = IN1,
GPIO18 = IN2, GPIO40 = PWM, GPIO38/39 = Hall A/B. The supply goes through the
INA228 to the DRI0042 "9-36V" terminal, and OUT1/OUT2 go to the motor.

### Door latch: solenoid TAU0730TM-14 via XY-MOS
12 V DC, 40 Ω, so it draws 0.3 A (3.6 W) when on.

```
 +12 V (supply side, BEFORE the INA228) ──► XY-MOS VIN+ ─┬─► OUT+ ──► solenoid wire 1
 GND (star point)                       ──► XY-MOS VIN− │   OUT− ──► solenoid wire 2
                                                         └── diode 1N5819 / 1N4007 across the
                                                             solenoid: stripe (cathode) to OUT+
 GPIO41 ──► XY-MOS TRIG/PWM          ESP32 GND ──► XY-MOS GND (next to TRIG)
 GPIO41 ──[10 kΩ]── GND   (pull-down: stays LOCKED while the ESP32 boots/flashes)
```

- **HIGH = solenoid ON = latch retracted = unlocked. LOW = OFF = extended = locked.**
  Power off, reset, reboot and a crash all leave it LOW, so the door stays **locked**.
- Take the 12 V from the supply side of the INA228 (not from `Current−`), so the
  0.3 A solenoid current does not show up as motor current and trigger the
  resistance detection.
- The diode is essential: without it, the voltage spike when the solenoid
  switches off damages the XY-MOS over time. Check whether your XY-MOS board
  already has one. Most don't.
- Mechanically, the latch must fall into its catch when the door is at the
  firmware's "closed" position (0°). Tune the timings in `app_config.h`
  (`APP_LATCH_*`).

### INA228
VCC → 3.3 V · SDA → GPIO1 · SCL → GPIO2 · GND → GND · Current+ ← supply +12 V ·
Current− → **motor red** · V+ / VBUS → Current− side · V− → GND.
Default address 0x40. Shunt "R002" = 0.002 Ω (`APP_INA228_SHUNT_OHMS`).
It measures **only the motor current**, which is what the safe-door logic needs.

## Power and grounding
- **3.3 V:** WM8960 logic, INA228, motor FG pull-up, ICS43434. Check the dev board regulator's budget.
- **5 V:** WM8960 speaker amp from its own buck converter.
- **12 V:** LED strips, tail light, eyes, door motor (red wire, through the INA228),
  latch solenoid (through the XY-MOS, not through the INA228).
- **Single star ground.** Route the motor PGND/COM return separately from the
  audio ground and join them only at the star point.

## Hardware notes the firmware relies on (recommended additions)

These are suggestions beyond the Revision 4 document. Each one prevents a
specific problem:

1. **Motor inputs at power-up.** Between power-on and the moment the firmware
   configures GPIO 18/21/40 (~100 ms), those pins float. The firmware sets PWM = 0
   and the brake on as its first action. A 10 kΩ pull-down on the PWM line makes
   "no speed" guaranteed even before that.
2. **100 kΩ pull-down on each 2N7000 gate** (GPIO 8–11). This keeps the tail
   lamps dark during reset instead of relying on the floating gate.
3. **WS2815 data level.** A WS2815 powered at 12 V wants ~3.5 V for a logic "1".
   A 3.3 V ESP32 signal usually works (the old build did), but if strips flicker or show
   random colours, add a 74AHCT125 level shifter (3.3 V → 5 V) on GPIO 4–7.
4. **On-board RGB LED.** Some ESP32-S3 DevKitC-1 v1.1 boards wire their RGB LED to
   **GPIO38** (= HALL_A). It is only a high-impedance input and does no harm, but
   the LED may flicker with the encoder signal. Many N16R8 clones use GPIO48 instead.
