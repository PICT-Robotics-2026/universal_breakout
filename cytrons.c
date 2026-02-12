#include <stdio.h>

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "cytrons.h"

static const char *TAG = "cytrons";
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES   LEDC_TIMER_11_BIT
#define LEDC_DUTY       (0)
#define LEDC_FREQUENCY  (20000)	// Frequency in Hertz

#define CYTRON_1_PWM_1_CHANNEL LEDC_CHANNEL_0
#define CYTRON_1_PWM_2_CHANNEL LEDC_CHANNEL_1
#define CYTRON_2_PWM_1_CHANNEL LEDC_CHANNEL_2
#define CYTRON_2_PWM_2_CHANNEL LEDC_CHANNEL_3
#define CYTRON_3_PWM_1_CHANNEL LEDC_CHANNEL_4
#define CYTRON_3_PWM_2_CHANNEL LEDC_CHANNEL_5

#define CYTRON_1_DIR_1_PIN 15
#define CYTRON_1_PWM_1_PIN 2
#define CYTRON_1_DIR_2_PIN 23
#define CYTRON_1_PWM_2_PIN 4

#define CYTRON_2_DIR_1_PIN 16
#define CYTRON_2_PWM_1_PIN 17
#define CYTRON_2_DIR_2_PIN 5
#define CYTRON_2_PWM_2_PIN 18

#define CYTRON_3_DIR_1_PIN 32
#define CYTRON_3_PWM_1_PIN 33
#define CYTRON_3_DIR_2_PIN 25
#define CYTRON_3_PWM_2_PIN 26

// first index is the cytron number - 1, second is motor number - 1
int motor_dir_pins[6] = {
    CYTRON_1_DIR_1_PIN, CYTRON_1_DIR_2_PIN,
    CYTRON_2_DIR_1_PIN, CYTRON_2_DIR_2_PIN,
    CYTRON_3_DIR_1_PIN, CYTRON_3_DIR_2_PIN,
};

// first index is the cytron number - 1, second is motor number - 1
int motor_pwm_pins[6] = {
    CYTRON_1_PWM_1_PIN, CYTRON_1_PWM_2_PIN,
    CYTRON_2_PWM_1_PIN, CYTRON_2_PWM_2_PIN,
    CYTRON_3_PWM_1_PIN, CYTRON_3_PWM_2_PIN,
 };

// first index is the cytron number - 1, second is motor number - 1
int motor_pwm_channels[6] = {
    CYTRON_1_PWM_1_CHANNEL, CYTRON_1_PWM_2_CHANNEL,
    CYTRON_2_PWM_1_CHANNEL, CYTRON_2_PWM_2_CHANNEL,
    CYTRON_3_PWM_1_CHANNEL, CYTRON_3_PWM_2_CHANNEL,
};

int motor_pwm_limits[6] = {
    512, 512, 512, 512, 512, 512
};

static int clamp(int input, int lower, int upper)
{
    
    if (upper <= lower)
	{
	    ESP_LOGE(TAG, "clamp upper limit %d is less equal to lower limit %d", upper, lower);
	    return 0;
	}
    
    if (input > upper)
	return upper;
    if (input < lower)
	return lower;

    return input;
}

bool is_timer_setup = false;
bool is_motor_setup[6] = { false, false, false, false, false, false };

static void setup_timer()
{
    if (is_timer_setup)
	return;

    ESP_LOGI("motor", "init timer");
    
    ledc_timer_config_t ledc_timer = {
	.speed_mode = LEDC_LOW_SPEED_MODE,
	.duty_resolution = LEDC_DUTY_RES,
	.timer_num = LEDC_TIMER,
	.freq_hz = LEDC_FREQUENCY,
	.clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    is_timer_setup = true;
}

static void setup_motor(int dir, int pwm, int ledc_channel_num)
{
    ledc_channel_config_t ledc_channel = {
	.speed_mode = LEDC_LOW_SPEED_MODE,
	.channel = ledc_channel_num,
	.timer_sel = LEDC_TIMER,
	.intr_type = LEDC_INTR_DISABLE,
	.gpio_num = pwm,
	.duty = 0,
	.hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    gpio_reset_pin(dir);
    gpio_set_direction(dir, GPIO_MODE_OUTPUT);
}

void motor_init(motor_t motor)
{
    if (is_motor_setup[motor])
	return;
    
    setup_timer();

    ESP_LOGI("motor", "init motor %d", motor);
    
    setup_motor(motor_dir_pins[motor],
		motor_pwm_pins[motor],
		motor_pwm_channels[motor]);

    is_motor_setup[motor] = true;
}

void motor_set_pwm_limit(motor_t motor, int speed_limit)
{
    motor_pwm_limits[motor] = abs(speed_limit);
}

void motor_set_speed(motor_t motor, int speed)
{
    /* to ensure that motor is initialized */
    motor_init(motor);
    
    int clamped_speed = clamp(speed,
			      -motor_pwm_limits[motor],
			      motor_pwm_limits[motor]);

    if (clamped_speed > 0)
	gpio_set_level(motor_dir_pins[motor], 1);
    else
	gpio_set_level(motor_dir_pins[motor], 0);

    int abs_speed = abs(clamped_speed);

    ledc_set_duty(LEDC_LOW_SPEED_MODE,
		  motor_pwm_channels[motor],
		  abs_speed);
    
    ledc_update_duty(LEDC_LOW_SPEED_MODE,
		     motor_pwm_channels[motor]);

}
