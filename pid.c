#include <stdio.h>

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include "cytrons.h"
#include "encoders.h"

bool is_at_least_one_pid_started = false; 

typedef struct pid_info
{
    bool pid_enabled;
    encoder_t encoder;
    float P;
    float I;
    float D;
    int target;
} pid_info;

pid_info pid_infos[4];

static void pid_loop()
{
    ESP_LOGI("pid_loop", "Starting PID loop task");
    while (true)
    {
	for (motor_t motor = 0; motor < 4; motor++)
	{
	    
	    pid_info info = pid_infos[motor];
	    if (!info.pid_enabled)
		continue;

	    int position = encoder_get_position(info.encoder);
	    int error = info.target - position;

	    int speed = info.P * error;

	    ESP_LOGI("pid_loop", "motor %d, speed: %d",
		     motor,
		     speed);
	    
	    motor_set_speed(motor, speed);
	}

	vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void pid_set_target(motor_t motor, int target)
{
    pid_infos[motor].target = target;
}

void pid_init()
{
    for (int i=0;i<4;i++)
    {
	pid_info p = {
	    .pid_enabled = false,
	    .encoder = 0,
	    .P = 0,
	    .I = 0,
	    .D = 0,
	    .target = 0
	};
	
	pid_infos[i] = p;
    }

    xTaskCreatePinnedToCore(pid_loop,
			    "pid_loop",
			    8192,
			    NULL,
			    5,
			    NULL,
			    1);
}

void pid_calibrate_encoder(motor_t motor, encoder_t encoder)
{

    encoder_init(encoder);

    motor_set_pwm_limit(motor, 2047);
    motor_set_speed(motor, 2047);
    
    vTaskDelay(pdMS_TO_TICKS(50));

    motor_set_pwm_limit(motor, 512);
    motor_set_speed(motor, 0);
    
    if (encoder_get_position(encoder) > 0)
    {
	encoder_set_direction(encoder, 1);
    }
    else if (encoder_get_position(encoder) < 0)
    {
	encoder_set_direction(encoder, -1);
    }
    else
    {
	ESP_LOGE("pid",
		 "ERROR!!! Motor M_%d, & Encoder E_%d Don't Match",
		 motor+1,
		 encoder+1);
    }
}

void pid_register_motor_encoder(motor_t motor,
				encoder_t encoder,
				float P)
{
    pid_calibrate_encoder(motor, encoder);

    pid_info info = {
	.pid_enabled = true,
	.encoder = encoder,
	.P = P,
	.I = 0.0,
	.D = 0.0,
	.target = 0
    };
    
    pid_infos[motor] = info;
}
