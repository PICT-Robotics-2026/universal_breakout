#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "limit_switches.h"

// these pins are doubled because two limit switches are on common
// pins cuz we dint have enough pins
#define LIMIT_SW_1_PIN 27
#define LIMIT_SW_2_PIN 27
#define LIMIT_SW_3_PIN 14
#define LIMIT_SW_4_PIN 14

int limit_switch_pins[4] = {
    LIMIT_SW_1_PIN,
    LIMIT_SW_2_PIN,
    LIMIT_SW_3_PIN,
    LIMIT_SW_4_PIN
};

bool are_switches_initialized = false;

void limit_switches_init()
{
    if (are_switches_initialized)
	return;
    
    // Configure the GPIO pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LIMIT_SW_1_PIN)|(1ULL << LIMIT_SW_2_PIN)|(1ULL << LIMIT_SW_3_PIN)|(1ULL << LIMIT_SW_3_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Enable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    are_switches_initialized = true;
}


// this function inverts the level because when the switch is pressed,
// it gives a LOW signal

int limit_get_pressed(limit_sw_t sw)
{
    limit_switches_init();
    
    return !gpio_get_level(limit_switch_pins[sw]);
}
