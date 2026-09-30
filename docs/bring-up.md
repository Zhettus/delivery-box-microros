# First power-on and calibration

Work through these steps in order the first time the firmware meets the real
hardware. Everything marked `[MEASURE]` in `main/app_config.h` is covered here.

> Open the console with `idf.py -p /dev/ttyACM0 monitor` and type the commands
> shown. Every command answers `OK …` or `ERR …`.

## 0. Before connecting the motor
- Leave the motor's OUT1/OUT2 disconnected for steps 1–4.
- Check the boot log summary. Every subsystem should say `OK`:
  ```
  subsystems: door=OK tail=OK eyes=OK leds=OK i2c=OK power=OK mic=OK speaker=OK
  ```
- `i2c scan` should list `0x1A` (WM8960) and `0x40` (INA228).
  - Nothing found: check the 4.7 kΩ pull-ups, 3.3 V, and SDA/SCL swapped.

## 1. LED strips
1. `leds identify`: strip 1 red, 2 green, 3 blue, 4 white. Pixel 0 of each
   strip (where its data wire enters) is magenta.
   - Wrong colours everywhere (red shows green): set `APP_LED_WIRE_ORDER_RGB 0`.
   - A strip is dark: check its data pin and 12 V. Flicker suggests a level shifter (see wiring.md).
2. Count the pixels and correct `APP_LED_STRIPn_PIXELS`.
3. `leds comet 0 255 255 3`: the comet must run **continuously** around the
   box. If it jumps, fix `APP_LED_LOOP`:
   - list the strips in the order the comet should visit them,
   - set `reversed = true` for a strip whose magenta pixel 0 is at the *end*
     of its side in that direction.
4. `power` while `leds solid 255 255 255` runs shows the real 12 V current.
   Adjust `APP_LED_MA_PER_PIXEL_WHITE` so `APP_LED_POWER_BUDGET_MA` limits correctly.

## 2. Tail light and eyes
- `tail brake 255`, `tail run 128`, `tail left 255`, `tail right 255`, `tail off`.
  If a lamp is on when it should be off, the 2N7000/P-FET stage is wired inverted.
- `eyes constant`, `eyes thinking 1.5`, `eyes cap 0.2` (brighter), `eyes off`.

## 3. Audio
1. Read the crystal on the WM8960 HAT (e.g. `24.000`, `24.576`, `12.288`) and
   set `APP_AUDIO_CODEC_MCLK_HZ`. A wrong value makes tones play at the wrong
   pitch or not at all.
2. `beep 440 1000` should play a clean A4 for one second. `volume 50` makes it quieter.
   The boot log must say `I2S0 full duplex (clocks from WM8960)`. The codec is
   the clock master, so a wrong crystal value shows up as a wrong pitch for both
   the speaker and the mic.
3. `mic 1000` in a quiet room gives roughly −70…−60 dBFS RMS. Clap: the peak
   should jump by 30 dB or more. A constant −138 dBFS means no data: check
   DOUT → GPIO17 and SEL → GND.

## 4. Power monitor
- `power`: the bus voltage should read your 12 V rail.
- If the current reads wrong by a constant factor, fix `APP_INA228_SHUNT_OHMS`
  (your board: "R002" = 0.002 Ω).

## 5. Door motor (brushless, built-in driver)
Motor **not attached to the door**, motor supply on:
`idf.py -DMOTOR_TEST=1 -p /dev/ttyACM0 build flash monitor`.

1. **Wire check** (automatic at start-up): prints which of blue/green/white
   is SPEED, DIR and BRAKE. Put the printed `#define` lines into
   `main/motor_test.c` and set `WIRE_CHECK 0`.
2. **Speed loop**: +5 rpm for 5 s, stop, −5 rpm for 5 s, stop, repeat. It
   prints target, measured speed, duty and position 4 times per second.
   If "+" turns the wrong way, flip `DIR_LEVEL_FORWARD`.
3. **Stall**: hold the shaft. After 0.6 s it prints `STALL` and stops.
4. Copy the same pins and levels into `main/board.h` / `main/app_config.h`
   (`PIN_MOTOR_*`, `APP_BLDC_*`) for the normal firmware.
5. **Latch solenoid** (normal firmware, door closed): `door unlock` should
   click the solenoid in, `door lock` let it out again after 100 ms.

## 6. Whole system
- `APP_DEMO_MODE 1` cycles through every state every 5 s (door stays still
  unless `APP_DEMO_MOVES_DOOR 1`).
- In normal mode, try every state: `states` lists them, and `state charging` etc. sets one.
