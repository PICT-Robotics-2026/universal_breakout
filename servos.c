#include <stdio.h>

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "servos.h"

static const char *TAG = "servos";
#define LEDC_TIMER      LEDC_TIMER_1
#define LEDC_MODE       LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES   LEDC_TIMER_13_BIT
#define LEDC_DUTY       (0)
#define LEDC_FREQUENCY  (50)	// Frequency in Hertz

#define SERVO_1_CHANNEL LEDC_CHANNEL_5
#define SERVO_2_CHANNEL LEDC_CHANNEL_6
#define SERVO_3_CHANNEL LEDC_CHANNEL_7

#define SERVO_1_PIN 12
#define SERVO_2_PIN 13
#define SERVO_3_PIN 19

int servo_pins[3] = {
    SERVO_1_PIN,
    SERVO_2_PIN,
    SERVO_3_PIN,
};

int servo_channels[3] = {
    SERVO_1_CHANNEL,
    SERVO_2_CHANNEL,
    SERVO_3_CHANNEL
};

int servo_min_us[3] = {500, 500, 500};
int servo_max_us[3] = {2500, 2500, 2500};
int servo_max_angle[3] = {180, 180, 180};

bool is_servo_initialized[3] = { false, false, false };

static int get_duty_cycle(int angle,
		   int min_duty_us,
		   int max_duty_us,
		   int max_angle)
{
    float min_duty = (float)min_duty_us / 20000.0 * 8192.0;
    float max_duty = (float)max_duty_us / 20000.0 * 8192.0;

    if (angle < 0)
	return min_duty;
    
    if (angle > max_angle)
	return max_duty;
   
    return ((float)angle / (float)max_angle * (float)(max_duty - min_duty) + min_duty);
}

void servo_set_angle(servo_t servo,
		     int angle)			
{
    /* ensure that servo is initialized */
    servo_init(servo);
    
    int duty_cycle = get_duty_cycle(angle,
				    servo_min_us[servo],
				    servo_max_us[servo],
				    servo_max_angle[servo]); 
    
    ledc_set_duty(LEDC_MODE, servo_channels[servo], duty_cycle); 
    ledc_update_duty(LEDC_MODE, servo_channels[servo]);	
}

void servo_set_limits(servo_t servo,
		      int min_duty_us,
		      int max_duty_us,
		      int max_angle)					
{									
    servo_min_us[servo] = min_duty_us;			
    servo_max_us[servo] = max_duty_us;			
    servo_max_angle[servo] = max_angle;				
}

bool is_timer_initialized = false;

static void servo_timer_init()
{
    if (is_timer_initialized)
	return;

    ESP_LOGI("servo", "init timer");
    
    ledc_timer_config_t ledc_timer = {
	.speed_mode       = LEDC_MODE,
	.duty_resolution  = LEDC_DUTY_RES,
	.timer_num        = LEDC_TIMER,
	.freq_hz          = LEDC_FREQUENCY, 
	.clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    is_timer_initialized = true;
}

void servo_init(servo_t servo)
{
    if (is_servo_initialized[servo])
	return;
    
    servo_timer_init();

    ESP_LOGI("servo", "init servo %d", servo);
    
    ledc_channel_config_t ledc_channel = {	
	.speed_mode     = LEDC_MODE,					
	.channel        = servo_channels[servo],			
	.timer_sel      = LEDC_TIMER,					
	.intr_type      = LEDC_INTR_DISABLE,				
	.gpio_num       = servo_pins[servo],			
	.duty           = 0,				
	.hpoint         = 0						
    };									
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel)); 

    is_servo_initialized[servo] = true;
}
