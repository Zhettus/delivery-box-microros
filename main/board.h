/*
 * board.h — ESP32-S3-WROOM-1 N16R8 pin map for the Delivery Box.
 *
 * THIS FILE IS THE SINGLE SOURCE OF TRUTH FOR PINS.
 * It mirrors §5 "Master pin map" of Delivery_Box_Wiring_N16R8 (Revision 4).
 * If the wiring changes, change it here and nowhere else.
 *
 * The build fails if a signal lands on a forbidden pin or two signals share
 * one pin (see the checks at the bottom).
 */
#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"

/* ---- Shared I2C bus: WM8960 (0x1A) + INA228 (0x40), 4.7k pull-ups ------ */
#define PIN_I2C_SDA         GPIO_NUM_1
#define PIN_I2C_SCL         GPIO_NUM_2

/* ---- WS2815 LED strips (one RMT channel each) ------------------------- */
#define PIN_LED_STRIP_1     GPIO_NUM_4
#define PIN_LED_STRIP_2     GPIO_NUM_5
#define PIN_LED_STRIP_3     GPIO_NUM_6
#define PIN_LED_STRIP_4     GPIO_NUM_7

/* ---- Tail light (2N7000 -> P-MOSFET per function, active HIGH) --------- */
#define PIN_TAIL_BRAKE      GPIO_NUM_8
#define PIN_TAIL_RUN        GPIO_NUM_9
#define PIN_TAIL_LEFT       GPIO_NUM_10
#define PIN_TAIL_RIGHT      GPIO_NUM_11

/* ---- Eyes (JZ-MOS TRIG/PWM) ------------------------------------------- */
#define PIN_EYE_LEFT        GPIO_NUM_12
#define PIN_EYE_RIGHT       GPIO_NUM_13

/* ---- Shared I2S (full duplex): WM8960 speaker + ICS43434 mic ----------- */
#define PIN_I2S_BCLK        GPIO_NUM_14   /* WM8960 P12  + ICS43434 BCLK */
#define PIN_I2S_WS          GPIO_NUM_15   /* WM8960 P35  + ICS43434 LRCL */
#define PIN_I2S_DOUT        GPIO_NUM_16   /* -> WM8960 P40 (DACDAT)      */
#define PIN_I2S_DIN         GPIO_NUM_17   /* <- ICS43434 DOUT            */

/* ---- Door motor ----------------------------------------------------------
 * Brushless (BLDC) gear motor WITH A BUILT-IN DRIVER. Its red/black wires go
 * straight to the supply (through the INA228); the ESP32 only talks to the
 * control wires. (The same pins serve a brushed motor + DRI0042 if
 * APP_DOOR_MOTOR_TYPE is switched: DIR = IN1, BRAKE = IN2, FG = Hall A,
 * GPIO39 = Hall B.) Mapping found with the wire finder (TEST_MODE 3).                                                         */
#define PIN_MOTOR_PWM       GPIO_NUM_18   /* GREEN: speed (PWM, HIGH = fast) */
#define PIN_MOTOR_DIR       GPIO_NUM_21   /* WHITE: direction (CW/CCW)       */
#define PIN_MOTOR_BRAKE     GPIO_NUM_40   /* BLUE:  brake (HIGH = brake)     */
#define PIN_MOTOR_FG        GPIO_NUM_38   /* YELLOW: FG speed pulses         */
#define PIN_MOTOR_AUX       GPIO_NUM_39   /* DRI0042 option only: Hall B    */

/* ---- Door latch ----------------------------------------------------------
 * Solenoid TAU0730TM-14 (12 V DC, 40 ohm = 0.3 A) through an XY-MOS module.
 *   HIGH = solenoid ON  = latch RETRACTED = door UNLOCKED
 *   LOW  = solenoid OFF = latch EXTENDED  = door LOCKED (also at reset/boot)
 * Put a 10 k pull-down from GPIO41 to GND so it stays locked while the
 * ESP32 boots or is being flashed.                                          */
#define PIN_DOOR_LATCH      GPIO_NUM_41   /* -> XY-MOS TRIG/PWM             */

/* Spare, untouched: GPIO 42, 47, 48. */

/* ------------------------------------------------------------------------ */
/* Peripheral allocation (so no two drivers ever claim the same unit)       */
/* ------------------------------------------------------------------------ */
/*
 * LEDC (ESP32-S3: 4 timers, 8 channels, low-speed mode only)
 *   timer 0 -> tail lights  channels 0..3
 *   timer 1 -> eyes         channels 4..5
 *   timer 2 -> door motor   channel  6     (20 kHz, above hearing)
 *   timer 3 -> door latch   channel  7     (solenoid, hold power)
 * RMT   : 4 TX channels, one per LED strip (all 4 the S3 has)
 * I2S   : I2S0, full duplex (TX speaker + RX microphone)
 * I2C   : I2C0 master
 * PCNT  : 1 unit for the door encoder
 */
#define LEDC_TIMER_TAIL     LEDC_TIMER_0
#define LEDC_CH_TAIL_BRAKE  LEDC_CHANNEL_0
#define LEDC_CH_TAIL_RUN    LEDC_CHANNEL_1
#define LEDC_CH_TAIL_LEFT   LEDC_CHANNEL_2
#define LEDC_CH_TAIL_RIGHT  LEDC_CHANNEL_3

#define LEDC_TIMER_EYES     LEDC_TIMER_1
#define LEDC_CH_EYE_LEFT    LEDC_CHANNEL_4
#define LEDC_CH_EYE_RIGHT   LEDC_CHANNEL_5

#define LEDC_TIMER_MOTOR    LEDC_TIMER_2
#define LEDC_CH_MOTOR       LEDC_CHANNEL_6

#define LEDC_TIMER_LATCH    LEDC_TIMER_3
#define LEDC_CH_LATCH       LEDC_CHANNEL_7

#define I2C_PORT_BOARD      I2C_NUM_0

/* ------------------------------------------------------------------------ */
/* Compile-time pin checks (wiring doc §3 "GPIO pins that are OFF LIMITS")  */
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

BOARD_CHECK_PIN(PIN_I2C_SDA);     BOARD_CHECK_PIN(PIN_I2C_SCL);
BOARD_CHECK_PIN(PIN_LED_STRIP_1); BOARD_CHECK_PIN(PIN_LED_STRIP_2);
BOARD_CHECK_PIN(PIN_LED_STRIP_3); BOARD_CHECK_PIN(PIN_LED_STRIP_4);
BOARD_CHECK_PIN(PIN_TAIL_BRAKE);  BOARD_CHECK_PIN(PIN_TAIL_RUN);
BOARD_CHECK_PIN(PIN_TAIL_LEFT);   BOARD_CHECK_PIN(PIN_TAIL_RIGHT);
BOARD_CHECK_PIN(PIN_EYE_LEFT);    BOARD_CHECK_PIN(PIN_EYE_RIGHT);
BOARD_CHECK_PIN(PIN_I2S_BCLK);    BOARD_CHECK_PIN(PIN_I2S_WS);
BOARD_CHECK_PIN(PIN_I2S_DOUT);    BOARD_CHECK_PIN(PIN_I2S_DIN);
BOARD_CHECK_PIN(PIN_MOTOR_PWM);   BOARD_CHECK_PIN(PIN_MOTOR_DIR);
BOARD_CHECK_PIN(PIN_MOTOR_BRAKE); BOARD_CHECK_PIN(PIN_MOTOR_FG);
BOARD_CHECK_PIN(PIN_MOTOR_AUX);   BOARD_CHECK_PIN(PIN_DOOR_LATCH);

/* No pin used twice: for distinct pins the sum of the bits equals their OR. */
#define B_(p) (1ULL << (p))
#define BOARD_PINS_SUM (B_(PIN_I2C_SDA) + B_(PIN_I2C_SCL) +                   \
    B_(PIN_LED_STRIP_1) + B_(PIN_LED_STRIP_2) + B_(PIN_LED_STRIP_3) +         \
    B_(PIN_LED_STRIP_4) + B_(PIN_TAIL_BRAKE) + B_(PIN_TAIL_RUN) +             \
    B_(PIN_TAIL_LEFT) + B_(PIN_TAIL_RIGHT) + B_(PIN_EYE_LEFT) +               \
    B_(PIN_EYE_RIGHT) + B_(PIN_I2S_BCLK) + B_(PIN_I2S_WS) +                   \
    B_(PIN_I2S_DOUT) + B_(PIN_I2S_DIN) + B_(PIN_MOTOR_DIR) +                  \
    B_(PIN_MOTOR_BRAKE) + B_(PIN_MOTOR_PWM) + B_(PIN_MOTOR_FG) + B_(PIN_MOTOR_AUX) +                    \
    B_(PIN_DOOR_LATCH))
#define BOARD_PINS_OR (B_(PIN_I2C_SDA) | B_(PIN_I2C_SCL) |                    \
    B_(PIN_LED_STRIP_1) | B_(PIN_LED_STRIP_2) | B_(PIN_LED_STRIP_3) |         \
    B_(PIN_LED_STRIP_4) | B_(PIN_TAIL_BRAKE) | B_(PIN_TAIL_RUN) |             \
    B_(PIN_TAIL_LEFT) | B_(PIN_TAIL_RIGHT) | B_(PIN_EYE_LEFT) |               \
    B_(PIN_EYE_RIGHT) | B_(PIN_I2S_BCLK) | B_(PIN_I2S_WS) |                   \
    B_(PIN_I2S_DOUT) | B_(PIN_I2S_DIN) | B_(PIN_MOTOR_DIR) |                  \
    B_(PIN_MOTOR_BRAKE) | B_(PIN_MOTOR_PWM) | B_(PIN_MOTOR_FG) | B_(PIN_MOTOR_AUX) |                    \
    B_(PIN_DOOR_LATCH))
_Static_assert(BOARD_PINS_SUM == BOARD_PINS_OR, "two signals share the same GPIO");
