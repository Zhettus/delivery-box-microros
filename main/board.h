/*
 * board.h — ESP32-S3-WROOM-1 N16R8 pin map for the Delivery Box.
 *
 * THIS FILE IS THE SINGLE SOURCE OF TRUTH FOR PINS.
 * If the wiring changes, change it here and nowhere else.
 *
 * The build fails if a signal lands on a forbidden pin or two signals share
 * one pin (see the checks at the bottom).
 */
#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"

/* ---- I2C bus, shared (wiring doc §4.1), 4.7k pull-ups to 3.3 V ----------
 *   INA228 motor-current sensor  0x40
 *   WM8960 Audio HAT (P3 SDA, P5 SCL)  0x1A                                 */
#define PIN_I2C_SDA         GPIO_NUM_1
#define PIN_I2C_SCL         GPIO_NUM_2

/* ---- I2S, one port (wiring doc §4.2, §6.4, §6.5) ------------------------
 * The ESP32 is the clock master; the WM8960 follows BCLK/WS.               */
#define PIN_I2S_BCLK        GPIO_NUM_14   /* WM8960 P12  + ICS43434 BCLK      */
#define PIN_I2S_WS          GPIO_NUM_15   /* WM8960 P35 (LRCLK) + ICS43434 LRCL */
#define PIN_I2S_DOUT        GPIO_NUM_16   /* WM8960 P40 (DACDAT): to the speaker */
#define PIN_I2S_DIN         GPIO_NUM_17   /* ICS43434 DOUT: microphone -> ESP32
                                             (ICS43434 SEL -> GND = left slot) */

/* ---- WS2815 LED strip (one strip, 134 LEDs) ---------------------------- */
#define PIN_LED_STRIP       GPIO_NUM_4

/* ---- Tail light (2N7000 -> P-MOSFET per function, HIGH = lamp on) ------ */
#define PIN_TAIL_BRAKE      GPIO_NUM_8
#define PIN_TAIL_RUN        GPIO_NUM_9
#define PIN_TAIL_LEFT       GPIO_NUM_10
#define PIN_TAIL_RIGHT      GPIO_NUM_11

/* ---- Face: eyes + mouth on one wire, one XY-MOS module (TRIG/PWM) ------ */
#define PIN_FACE            GPIO_NUM_12   /* eyes + mouth, one XY-MOS TRIG/PWM */

/* ---- Door motor ----------------------------------------------------------
 * Brushless (BLDC) gear motor WITH A BUILT-IN DRIVER. Its red/black wires go
 * straight to the supply (through the INA228); the ESP32 only talks to the
 * control wires.                                                            */
#define PIN_MOTOR_PWM       GPIO_NUM_18   /* motor speed input (PWM)          */
#define PIN_MOTOR_DIR       GPIO_NUM_21   /* motor direction input (CW/CCW)   */
#define PIN_MOTOR_BRAKE     GPIO_NUM_40   /* motor brake input (HIGH = brake) */
/* PWM and BRAKE are the other way round from the esp32s3 project's wiring
 * doc (PWM 40, BRAKE 18). Measured on the box on 2026-10-01: GPIO18 duty
 * sets the speed (10 % = 3 rpm, 100 % = 49 rpm), GPIO40 HIGH stops it.   */
#define PIN_MOTOR_FG        GPIO_NUM_38   /* motor FG speed pulses (yellow)   */

/* ---- Door latch ----------------------------------------------------------
 * Solenoid through an XY-MOS module.
 *   HIGH = solenoid ON  = latch RETRACTED = door UNLOCKED
 *   LOW  = solenoid OFF = latch EXTENDED  = door LOCKED (also at reset/boot) */
#define PIN_DOOR_LATCH      GPIO_NUM_41

/* ---- Door-closed switch --------------------------------------------------
 * One side to GPIO5, the other to GND (internal pull-up; a 10 k pull-up to
 * 3.3 V makes it sturdier on long wires).
 *   door CLOSED = switch closed = LOW      door OPEN = switch open = HIGH   */
#define PIN_DOOR_SWITCH     GPIO_NUM_5

/* Not used by this firmware: GPIO 6, 7, 13, 39, 42, 47, 48. */

/* ------------------------------------------------------------------------ */
/* Peripheral allocation (so no two drivers ever claim the same unit)       */
/* ------------------------------------------------------------------------ */
/*
 * LEDC (ESP32-S3: 4 timers, 8 channels)
 *   timer 0 -> face         channel  0
 *   timer 1 -> door motor   channel  3     (20 kHz, above hearing)
 *   timer 2 -> door latch   channel  4     (solenoid, hold power)
 * RMT   : 1 TX channel for the LED strip
 * I2C   : I2C0 master (INA228 + WM8960)
 * I2S   : I2S0 full duplex: TX -> WM8960, RX <- ICS43434
 * PCNT  : 1 unit for the motor FG pulses
 * The tail lights are plain on/off GPIOs.
 */
#define LEDC_TIMER_FACE     LEDC_TIMER_0
#define LEDC_CH_FACE        LEDC_CHANNEL_0

#define LEDC_TIMER_MOTOR    LEDC_TIMER_1
#define LEDC_CH_MOTOR       LEDC_CHANNEL_3

#define LEDC_TIMER_LATCH    LEDC_TIMER_2
#define LEDC_CH_LATCH       LEDC_CHANNEL_4

#define I2C_PORT_BOARD      I2C_NUM_0

/* ------------------------------------------------------------------------ */
/* Compile-time pin checks                                                  */
/* ------------------------------------------------------------------------ */
#define BOARD_PIN_FORBIDDEN(p) (                                              \
    ((p) >= 22 && (p) <= 25) ||  /* not bonded out on the ESP32-S3         */ \
    ((p) >= 26 && (p) <= 32) ||  /* internal SPI flash                     */ \
    ((p) >= 33 && (p) <= 37) ||  /* internal octal PSRAM (R8)              */ \
    (p) == 0 ||                  /* BOOT button / strapping                */ \
    (p) == 3 || (p) == 45 || (p) == 46 ||  /* strapping                    */ \
    (p) == 19 || (p) == 20 ||    /* native USB D-/D+                       */ \
    (p) == 43 || (p) == 44)      /* UART0 TX/RX: flashing + serial monitor */

#define BOARD_CHECK_PIN(p) \
    _Static_assert(!BOARD_PIN_FORBIDDEN(p), #p " is on a forbidden GPIO (see board.h)")

BOARD_CHECK_PIN(PIN_I2C_SDA);      BOARD_CHECK_PIN(PIN_I2C_SCL);
BOARD_CHECK_PIN(PIN_I2S_BCLK);     BOARD_CHECK_PIN(PIN_I2S_WS);
BOARD_CHECK_PIN(PIN_I2S_DOUT);     BOARD_CHECK_PIN(PIN_I2S_DIN);
BOARD_CHECK_PIN(PIN_LED_STRIP);
BOARD_CHECK_PIN(PIN_TAIL_BRAKE);   BOARD_CHECK_PIN(PIN_TAIL_RUN);
BOARD_CHECK_PIN(PIN_TAIL_LEFT);    BOARD_CHECK_PIN(PIN_TAIL_RIGHT);
BOARD_CHECK_PIN(PIN_FACE);
BOARD_CHECK_PIN(PIN_MOTOR_PWM);    BOARD_CHECK_PIN(PIN_MOTOR_DIR);
BOARD_CHECK_PIN(PIN_MOTOR_BRAKE);  BOARD_CHECK_PIN(PIN_MOTOR_FG);
BOARD_CHECK_PIN(PIN_DOOR_LATCH);   BOARD_CHECK_PIN(PIN_DOOR_SWITCH);

/* No pin used twice: for distinct pins the sum of the bits equals their OR. */
#define B_(p) (1ULL << (p))
#define BOARD_PINS_SUM (B_(PIN_I2C_SDA) + B_(PIN_I2C_SCL) +                   \
    B_(PIN_I2S_BCLK) + B_(PIN_I2S_WS) + B_(PIN_I2S_DOUT) + B_(PIN_I2S_DIN) +  \
    B_(PIN_LED_STRIP) + B_(PIN_TAIL_BRAKE) + B_(PIN_TAIL_RUN) +             \
    B_(PIN_TAIL_LEFT) + B_(PIN_TAIL_RIGHT) + B_(PIN_FACE) +                   \
    B_(PIN_MOTOR_PWM) +         \
    B_(PIN_MOTOR_DIR) + B_(PIN_MOTOR_BRAKE) + B_(PIN_MOTOR_FG) +              \
    B_(PIN_DOOR_LATCH) + B_(PIN_DOOR_SWITCH))
#define BOARD_PINS_OR (B_(PIN_I2C_SDA) | B_(PIN_I2C_SCL) |                    \
    B_(PIN_I2S_BCLK) | B_(PIN_I2S_WS) | B_(PIN_I2S_DOUT) | B_(PIN_I2S_DIN) |  \
    B_(PIN_LED_STRIP) | B_(PIN_TAIL_BRAKE) | B_(PIN_TAIL_RUN) |             \
    B_(PIN_TAIL_LEFT) | B_(PIN_TAIL_RIGHT) | B_(PIN_FACE) |                   \
    B_(PIN_MOTOR_PWM) |         \
    B_(PIN_MOTOR_DIR) | B_(PIN_MOTOR_BRAKE) | B_(PIN_MOTOR_FG) |              \
    B_(PIN_DOOR_LATCH) | B_(PIN_DOOR_SWITCH))
_Static_assert(BOARD_PINS_SUM == BOARD_PINS_OR, "two signals share the same GPIO");
