/*
 * app_config.h — Every tunable value of the Delivery Box in one place.
 *
 * Pins live in board.h. Robot-state looks (colours, eye moods, tail
 * lights, sounds, door action per state) live in robot_state.c.
 *
 * Values marked  [MEASURE]  depend on your mechanics/hardware and should be
 * checked during bring-up (see docs/bring-up.md).
 */
#pragma once

/* ======================================================================== */
/* Application                                                              */
/* ======================================================================== */

/* 1 = cycle through all robot states forever (bench demo, like the old
 *     final.c). 0 = wait for commands on the serial port (normal use). */
#define APP_DEMO_MODE                 0      /* ROS version: the robot computer sets the states */
#define APP_DEMO_STEP_MS              5000
#define APP_DEMO_MOVES_DOOR           1      /* 1 = demo also opens/closes the door */

/* ======================================================================== */
/* micro-ROS (ROS 2 Jazzy) over the native USB port — see ros_interface.h   */
/* ======================================================================== */
#define APP_ROS_NODE_NAME             "delivery_box"
/* If the ROS agent disappears (robot computer crashed, cable pulled):      */
#define APP_ROS_STOP_DOOR_WHEN_LOST   1      /* stop the door if it is moving  */
#define APP_ROS_WARN_WHEN_LOST        0      /* 1 = also show failure_not_critical */

/* Echo typed characters back on the serial port (handy for humans in
 * `idf.py monitor`; set 0 if a program drives the port). */
#define APP_SERIAL_ECHO               1

/* ======================================================================== */
/* I2C bus (WM8960 + INA228)                                                */
/* ======================================================================== */
#define APP_I2C_SPEED_HZ              400000

/* ======================================================================== */
/* LED strips — WS2815, 12 V                                                */
/* ======================================================================== */
/* Pixel count per strip. Strip n is on PIN_LED_STRIP_n.            [MEASURE]
 * Taken from the old firmware: two 29-pixel sides, 19-pixel front/back. */
#define APP_LED_STRIP1_PIXELS         29     /* left side  */
#define APP_LED_STRIP2_PIXELS         29     /* right side */
#define APP_LED_STRIP3_PIXELS         19     /* front      */
#define APP_LED_STRIP4_PIXELS         19     /* back       */

/* The loop around the box used by the comet and fill effects: the order
 * the strips are walked in, and whether each strip runs backwards along
 * that walk. Use `leds identify` to see where pixel 0 of each strip is. [MEASURE]
 * Format: { strip index (0 = strip 1), reversed } */
#define APP_LED_LOOP                  { { 0, false }, { 2, false }, \
                                        { 1, true  }, { 3, true  } }

#define APP_LED_WIRE_ORDER_RGB        1      /* WS2815 = RGB (old firmware swapped R/G) */
#define APP_LED_MAX_BRIGHTNESS        220    /* 0..255                         */
#define APP_LED_COLOR_CORRECTION      LED_RGB(0xFF, 0xB0, 0xF0)  /* "TypicalLEDStrip" */
#define APP_LED_TRANSITION_S          0.0f   /* 0 = fade out over the new state's time (as the original) */
#define APP_LED_FRAME_MS              10     /* 100 fps                        */
#define APP_LED_POWER_BUDGET_MA       2500   /* 12 V current allowed for strips */
#define APP_LED_MA_PER_PIXEL_WHITE    20.0f  /* ~14.4 W/m @ 60 px/m        [MEASURE] */
#define APP_LED_MA_PER_PIXEL_IDLE     1.0f
#define APP_LED_COMET_TAIL_PX         10.0f
#define APP_LED_DIVERGE_SPEED_PX_S    12.0f  /* from the old led-check.c        */
#define APP_LED_DIVERGE_TAIL_PX       7.0f

/* ======================================================================== */
/* Eyes — JZ-MOS drivers                                                    */
/* ======================================================================== */
#define APP_EYES_PWM_HZ               1500
#define APP_EYES_MIN_PULSE_US         20     /* JZ-MOS ignores shorter pulses  */
#define APP_EYES_GAMMA                2.2f
/* Brightest the eyes ever get (0..1). The eyes never go below the minimum
 * pulse (20 us at 1500 Hz = 3 %), so fades and breathing happen between 3 %
 * and this cap. The original final.c used 0.05 -> only 3..5 %, too small to
 * see; the original eyes_check.c used 0.5. Live: `eyes cap 0.3`.       */
#define APP_EYES_BRIGHTNESS_CAP       0.50f
#define APP_EYES_INVERT               0      /* 1 if the JZ-MOS lights on LOW    */

/* ======================================================================== */
/* Tail light                                                               */
/* ======================================================================== */
/* Low PWM frequency on purpose: the P-MOSFET gate is pulled up through
 * 10 kOhm, so it switches off slowly (tens of us). 500 Hz keeps those
 * edges a small part of each period. */
#define APP_TAIL_PWM_HZ               500
#define APP_TAIL_LEVEL_ON             255    /* "on" brightness used by states */

/* ======================================================================== */
/* Door motor                                                               */
/* ======================================================================== */
/* 0 = brushless motor with built-in driver (PWM / DIR / BRAKE / FG wires)
 * 1 = brushed motor + DFRobot DRI0042 + 2-channel encoder                  */
#define APP_DOOR_MOTOR_TYPE           0

/* ---- Brushless motor with built-in driver ----------------------- [MEASURE]
 * Run the identify test (docs/bring-up.md) to find these values.          */
#define APP_BLDC_PWM_HZ               20000  /* identify test: speed follows duty linearly */
#define APP_BLDC_PWM_INVERTED         0      /* wire check: more duty = faster  */
#define APP_BLDC_DIR_LEVEL_OPEN       0      /* DIR level that opens the door (tested: 0) */
#define APP_BLDC_HAS_BRAKE            1      /* 0 if the motor has no brake wire */
#define APP_BLDC_BRAKE_LEVEL_ON       1      /* BRAKE level that brakes (wire check: HIGH) */
/* FG pulses per OUTPUT-shaft revolution (motor FG pulses/rev x gear ratio).
 * Measure: identify test, count output turns, divide.                     */
#define APP_BLDC_FG_PER_OUTPUT_REV    480.0f /* FG count: ~1920 pulses / 4 turns (refine with the exact total) */
#define APP_BLDC_OUTPUT_RPM_FULL      48.0f  /* 384.6 Hz FG x 60 / 480            */
#define APP_BLDC_MIN_DUTY             0.10f  /* identify test: turns at 10 %      */
#define APP_BLDC_STOP_BEFORE_REV_MS   500    /* stop time before reversing       */

/* ---- Brushed motor + DRI0042 (APP_DOOR_MOTOR_TYPE 1) --------------------- */
#define APP_DRI0042_PWM_HZ            20000  /* §7.3: 20-25 kHz, no audible whine */
#define APP_DRI0042_MIN_PULSE_NS      200
#define APP_DRI0042_INVERT_MOTOR      0
#define APP_DRI0042_INVERT_ENCODER    0
#define APP_DRI0042_ENCODER_PPR       16     /* pulses / motor rev / channel     */
#define APP_DRI0042_GEAR_RATIO        131.0f
#define APP_DRI0042_MOTOR_RPM_FULL    4500.0f
#define APP_DRI0042_MIN_DUTY          0.15f
#define APP_DRI0042_STOP_BEFORE_REV_MS 150   /* datasheet: >= 100 ms              */

/* ---- Speed control (both motor types, at the OUTPUT shaft) ---------------- */
#define APP_DOOR_KP                   0.012f /* duty per output rpm of error      */
#define APP_DOOR_KI                   0.06f  /* duty per (output rpm x s)         */
#define APP_DOOR_INTEGRAL_LIMIT       0.40f
#define APP_DOOR_SPEED_WINDOW_MS      200    /* speed measured over this window   */
#define APP_DOOR_CONTROL_MS           20     /* 50 Hz control loop                */

/* Door travel (output shaft)                                        [MEASURE] */
#define APP_DOOR_OPEN_DEG             90.0f
#define APP_DOOR_TOLERANCE_DEG        2.0f
#define APP_DOOR_SLOWDOWN_DEG         20.0f
#define APP_DOOR_CRUISE_RPM           5.0f
#define APP_DOOR_APPROACH_RPM         3.0f
#define APP_DOOR_HOMING_RPM           3.0f
#define APP_DOOR_HOME_AT_BOOT         0      /* 0 = assume closed at power-up  */

/* Door moves made by the robot states (main/robot_state.c):
 *   success_open_door   -> go TO APP_DOOR_OPEN_DEG at OPEN_RPM, stays open
 *   success_close_door  -> go TO 0 (closed)        at CLOSE_RPM, latch locks
 *   moving_* states     -> go TO 0 (closed)        at CLOSE_RPM
 * Each with its own current limit: closing may catch a hand, so it can
 * be stricter than opening.                                          [MEASURE] */
#define APP_DOOR_OPEN_RPM             5.0f   /* motor test: smooth and steady */
#define APP_DOOR_CLOSE_RPM            5.0f
#define APP_DOOR_OPEN_LIMIT_A         APP_DOOR_CURRENT_LIMIT_A
#define APP_DOOR_CLOSE_LIMIT_A        APP_DOOR_CURRENT_LIMIT_A

/* Resistance from the MOTOR CURRENT (INA228). A limit of 0 switches it off.
 * Resistance = current above LIMIT, or RISE above the normal running
 * current, for CONFIRM_MS.                                             [MEASURE] */
#define APP_DOOR_CURRENT_LIMIT_A      0.50f
#define APP_DOOR_CURRENT_RISE_A       0.10f
#define APP_DOOR_CURRENT_CONFIRM_MS   120
#define APP_DOOR_CURRENT_IGNORE_MS    400    /* start-up spike of every move    */

/* Interference handling: move -> interference -> go back -> wait 3 s -> retry */
#define APP_DOOR_SPINUP_GRACE_MS      500    /* no detection right after start */
#define APP_DOOR_OBSTRUCTION_MS       300    /* too slow for this long = blocked */
#define APP_DOOR_OBSTRUCTION_RATIO    0.35f  /* too slow = < 35 % of commanded */
#define APP_DOOR_RETRY_WAIT_MS        3000
#define APP_DOOR_MAX_RETRIES          3
#define APP_DOOR_MOVE_TIMEOUT_MS      10000

/* ======================================================================== */
/* Door latch — solenoid TAU0730TM-14 via XY-MOS (board.h PIN_DOOR_LATCH)   */
/* ======================================================================== */
/*
 *   LOCKED ----open/any move----> UNLOCKING (solenoid ON, motor waits)
 *     ^                               | APP_LATCH_UNLOCK_MS
 *     |                               v
 *   LOCKING <--back at closed--- OPENING / OPEN / CLOSING (solenoid ON)
 *     (motor off, APP_LATCH_SETTLE_MS, then solenoid OFF)
 *
 * The door auto-closes APP_DOOR_HOLD_OPEN_MS after it has reached a
 * position other than closed (0 = stay open until a close command).
 */
#define APP_LATCH_ENABLED             1      /* 0 = no solenoid fitted          */
#define APP_LATCH_UNLOCK_MS           200    /* latch fully retracted after this */
#define APP_LATCH_SETTLE_MS           100    /* door seats, then latch extends  */
/* Power after the first APP_LATCH_PULL_IN_MS: 1.0 = full 12 V all the time
 * (3.6 W, the solenoid gets warm if the door stays open long). A solenoid
 * needs much less power to HOLD than to pull in: try 0.5 once it works. */
#define APP_LATCH_PULL_IN_MS          300
#define APP_LATCH_HOLD_DUTY           1.0f
#define APP_LATCH_PWM_HZ              1000   /* XY-MOS handles up to 20 kHz     */

#define APP_DOOR_HOLD_OPEN_MS         0      /* auto-close after this; 0 = stay open
                                                (success_close_door closes it) */

/* ======================================================================== */
/* Audio — WM8960 HAT + ICS43434                                            */
/* ======================================================================== */
#define APP_AUDIO_CODEC_ADDRESS       0x1A   /* WM8960 (fixed)                 */
/* 32 kHz: the ICS43434 needs >= ~23 kHz (BCLK >= 1.47 MHz); WM8960 supports
 * 8/12/16/24/32/48 kHz from its 12.288 MHz SYSCLK. */
#define APP_AUDIO_SAMPLE_RATE_HZ      32000
/* Crystal on the WM8960 HAT. Read the marking on the crystal:     [MEASURE]
 * 24.000 MHz (Waveshare), 24.576 MHz or 12.288 MHz all work. */
#define APP_AUDIO_CODEC_MCLK_HZ       24000000
#define APP_AUDIO_VOLUME_PERCENT      75
#define APP_AUDIO_TONE_AMPLITUDE      0.35f  /* 0..1 digital level of beeps    */
#define APP_AUDIO_STATE_SOUNDS        0      /* 1 = play a sound on state changes (off: states are silent) */

/* ======================================================================== */
/* Power monitor — INA228                                                   */
/* ======================================================================== */
#define APP_INA228_ADDRESS            0x40   /* A0 = A1 = GND                  */
#define APP_INA228_SHUNT_OHMS         0.002f /* on-board shunt marked "R002" = 2 mOhm */
#define APP_INA228_MAX_CURRENT_A      20.0f  /* 20 A x 2 mOhm = 40 mV -> fine 40.96 mV range */
#define APP_POWER_PERIOD_MS           1000
#define APP_POWER_LOG_EVERY_S         60     /* 0 = never log periodically     */
#define APP_POWER_WARN_LOW_V          10.5f  /* 12 V rail sagging / battery low */
#define APP_POWER_WARN_HIGH_A         8.0f
