#include "driver/gpio.h"

#include "board.h"
#include "tail_light.h"

static void config_pin(gpio_num_t pin)
{
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
}

void tail_lights_init(void)
{
    config_pin(PIN_TAIL_RUN);
    config_pin(PIN_TAIL_BRAKE);
    config_pin(PIN_TAIL_LEFT);
    config_pin(PIN_TAIL_RIGHT);
}

void tail_lights_set_running(bool enable)
{
    gpio_set_level(PIN_TAIL_RUN, enable ? 1 : 0);
}

void tail_lights_set_brake(bool enable)
{
    gpio_set_level(PIN_TAIL_BRAKE, enable ? 1 : 0);
}

void tail_lights_set_left_turn(bool enable)
{
    gpio_set_level(PIN_TAIL_LEFT, enable ? 1 : 0);
}

void tail_lights_set_right_turn(bool enable)
{
    gpio_set_level(PIN_TAIL_RIGHT, enable ? 1 : 0);
}

void tail_lights_off_all(void)
{
    tail_lights_set_running(false);
    tail_lights_set_brake(false);
    tail_lights_set_left_turn(false);
    tail_lights_set_right_turn(false);
}
