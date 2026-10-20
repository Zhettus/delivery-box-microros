/*
 * sound.h — What the box says, on the WM8960 Audio HAT speaker.
 *
 * Every sound is queued and plays in the background; the call returns at
 * once. A new sound stops the one that is playing. Audio streamed from the
 * computer (ros_interface.h, speaker/audio) plays through the same speaker. Without the HAT (or with
 * AUDIO_ENABLED 0 in config.h) the box simply stays silent.
 *
 *   sound           what it is                    used by (states.c / main.c)
 *   SOUND_STARTUP   three rising notes            start_up
 *   SOUND_ARRIVED   two notes                     success_waiting
 *   SOUND_GREETING  main/gm.wav                   success_open_door
 *   SOUND_WARNING   two short beeps               failure_not_critical
 *   SOUND_FAILURE   falling notes                 failure
 *   SOUND_DOOR_HELD fast beeps                    someone held the door
 */
#pragma once

#include <stdbool.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef enum {
    SOUND_STARTUP = 0,
    SOUND_ARRIVED,
    SOUND_GREETING,
    SOUND_WARNING,
    SOUND_FAILURE,
    SOUND_DOOR_HELD,
} sound_t;

/** Configure the WM8960 on the shared I2C `bus`, then start I2S (speaker +
 *  microphone). ESP_ERR_NOT_FOUND = no HAT: silent, but the mic still works. */
esp_err_t sound_init(i2c_master_bus_handle_t bus);

void sound_play(sound_t s);

/** Stop the sound that plays, and the computer's audio stream. */
void sound_stop(void);

/** "startup", "arrived", "greeting", "warning", "failure", "door_held". */
bool sound_from_name(const char *name, sound_t *out);

/** Speaker volume 0..100 (start-up value: AUDIO_VOLUME_PERCENT). */
void sound_set_volume(uint8_t percent);
