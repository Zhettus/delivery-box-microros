/*
 * face.h — The face: eyes and mouth on one wire, behind one XY-MOS module.
 *
 * A background task renders the current "emotion" with smooth transitions:
 *
 *   FACE_OFF        fade to dark over fade_s
 *   FACE_CONSTANT   fade to full brightness over fade_s, then stay on
 *   FACE_THINKING   slow "breathing": in over fade_s, out over fade_s, repeat
 *
 * face_set_level() takes manual control (e.g. to follow an LED animation)
 * until the next face_set_emotion().
 */
#pragma once

#include "esp_err.h"

#define FACE_FADE_MIN  0.01f    /* shortest fade time, seconds */

typedef enum {
    FACE_OFF = 0,
    FACE_THINKING,
    FACE_CONSTANT,
} face_emotion_t;

/** Set up the PWM output (dark) and start the background task. */
esp_err_t face_init(void);

/** Change the emotion, continuing smoothly from the current brightness. */
void face_set_emotion(face_emotion_t e, float fade_s);

/** Manual brightness 0..1, until the next face_set_emotion(). */
void face_set_level(float level);

/** Brightest the face ever gets, 0..1 (start-up value: FACE_BRIGHTNESS_CAP). */
void face_set_brightness_cap(float cap);
