#include <stdio.h>

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include "cytrons.h"
#include "encoders.h"

#define max(x,y) (((x) > (y)) ? (x) : (y))

bool is_pid_thread_started = false; 
bool is_pid_initialized = false;

typedef struct pid_info
{
    bool pid_enabled;
    encoder_t encoder;
    float P;
    float I;
    float D;
    int target;
    int history_ptr;
} pid_info;

pid_info pid_infos[6];
int last_encoder_readings[6] = {0};

/* Here we define an abstract concept called 'stall' stall will be
   proportional to the motor PWM and inversely proportional to the
   derivative of the encoder reading. The exact formula becomes 

   Stall = PWM / (1 + max(dx/dt, 0))

   PWM - motor pwm
   dx/dt - change in the encoder reading since last loop iteration

   We will maintain a history of the 'stall' quantity for the past If
   the average value of this goes above a certain threshold which is
   determined exprimentally, the motor will be stopped to prevent
   excessive stalling

*/

/* 40 readings corresponds to 2 seconds as our loop is 20hz*/

#define STALL_HISTORY_SIZE 40	
#define MAX_STALL 1000

int stall_history[6][STALL_HISTORY_SIZE];

int get_avg_stall(motor_t motor)
{
    float sum = 0;
    for (int i = 0; i < STALL_HISTORY_SIZE; i++)
    {
	sum += stall_history[motor][i];
    }

    return (int)(sum/STALL_HISTORY_SIZE);
}

int get_stall(int pwm, int last_encoder_reading, int current_encoder_reading)
{
    int pwm_sign = (pwm >= 0) ? 1 : -1;
    float d_x = current_encoder_reading - last_encoder_reading;
    return  abs((int)((float)pwm / (float) (1 + max(d_x * pwm_sign, 0))));
}

static void pid_loop()
{
    if (is_pid_thread_started)
	return;

    is_pid_thread_started = true;
    
    ESP_LOGI("pid_loop", "Starting PID loop task");
    while (true)
    {
	for (motor_t motor = 0; motor < 6; motor++)
	{
	    
	    pid_info info = pid_infos[motor];
	    if (!info.pid_enabled)
		continue;

	    int position = encoder_get_position(info.encoder);
	    int error = info.target - position;

	    int speed = info.P * error;

	    int stall = get_stall(speed, last_encoder_readings[motor], position);

	    stall_history[motor][info.history_ptr % STALL_HISTORY_SIZE] = stall;
	    pid_infos[motor].history_ptr += 1;
	    last_encoder_readings[motor] = position;


	    
	    /* ESP_LOGI("pid_loop", "motor %d, speed: %d", */
	    /* 	     motor, */
	    /* 	     speed); */

	    int avg_stall = get_avg_stall(motor);

	    ESP_LOGI("stall", "avg_stall: %d", avg_stall);
	    
	    if (avg_stall > MAX_STALL)
	    {
		motor_set_speed(motor, 0);
		ESP_LOGE("motor", "Motor stall detected, stopping motor %d", motor + 1);
	    }
	    else
	    {
		motor_set_speed_smooth(motor, speed);
	    }
	}

	vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void pid_goto(motor_t motor, int target)
{
    pid_infos[motor].target = target;
}

void pid_init()
{
    if (is_pid_initialized)
	return;

    is_pid_initialized = true;
    
    for (int i=0;i<6;i++)
    {
	pid_info p = {
	    .pid_enabled = false,
	    .encoder = 0,
	    .P = 0,
	    .I = 0,
	    .D = 0,
	    .target = 0,
	    .history_ptr = 0
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
    pid_init();

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

void pid_register(motor_t motor,
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
