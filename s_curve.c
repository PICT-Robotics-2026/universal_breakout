#include <stdio.h>
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include<math.h>
#include "encoders.h"
#include "pid.h"

typedef struct s_curve_info
{
    bool s_curve_enabled;
    int start_pos;
    int max_speed;
    encoder_t encoder;
    float A;
    float c;
    float h;
} s_curve_info;

s_curve_info s_curve_infos[6];

bool is_s_curve_task_started = false;

static float sigmoid(motor_t motor)
{
    int64_t t = esp_timer_get_time()/1000;
    float result =  s_curve_infos[motor].start_pos + s_curve_infos[motor].A/(1.0 + exp(-s_curve_infos[motor].h * (t -  s_curve_infos[motor].c)));
    return result;
}

static void s_curve_task()
{
    if (is_s_curve_task_started)
    {
	return;
    }

    is_s_curve_task_started = true;

    while(true)
    {
	for(motor_t motor = 0;motor < 6;motor++)
	{
	    s_curve_info info = s_curve_infos[motor];
	    if(!info.s_curve_enabled)
	    {
		continue;
	    }
	    float new_set_point = sigmoid(motor);
	    pid_goto(motor,new_set_point);
	    // ESP_LOGI("s_curve","motor: %d , tar: %f, A:%f, c:%f, h: %f",motor+1,new_set_point, info.A, info.c, info.h);
	}

	vTaskDelay(pdMS_TO_TICKS(7));
    }
    
}

void s_curve_register(motor_t motor, encoder_t encoder, int max_speed)
{
    s_curve_infos[motor].s_curve_enabled = true;
    s_curve_infos[motor].max_speed = max_speed;
    s_curve_infos[motor].encoder = encoder;
    s_curve_infos[motor].start_pos = encoder_get_position(encoder);
    s_curve_infos[motor].A = 0;
    
}

void s_curve_init()
{
    for(motor_t motor = 0;motor < 6; motor++)
    {
	s_curve_info info = {
	    .s_curve_enabled = false,
	    .max_speed = 0,
	    .A = 0,
	    .c = 0,
	    .h = 0,
	    .start_pos = 0,
	    .encoder = 0
	};

	s_curve_infos[motor] = info;
    }

    xTaskCreatePinnedToCore(s_curve_task,
			    "s_curve_task",
			    8192,
			    NULL,
			    5,
			    NULL,
			    1);
}



void s_curve_goto(motor_t motor, int new_position)
{
    vTaskDelay(pdMS_TO_TICKS(1));
    
    int current_position = encoder_get_position(s_curve_infos[motor].encoder);
    s_curve_infos[motor].A = new_position - current_position;
    s_curve_infos[motor].h = ((float)s_curve_infos[motor].max_speed)/(fabsf(s_curve_infos[motor].A) * 250.0);
    s_curve_infos[motor].c = (esp_timer_get_time() / 1000) + (4.0/s_curve_infos[motor].h);
    s_curve_infos[motor].start_pos = current_position;

    s_curve_info info = s_curve_infos[motor];
    
    ESP_LOGI("s_curve","motor: %d , A:%f, c:%f, h: %f",motor+1, info.A, info.c, info.h);
}

