#include <stdio.h>

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include "cytrons.h"
#include "encoders.h"
#include "limit_switches.h"
#include "leds.h"

#define max(x,y) (((x) > (y)) ? (x) : (y))

#define BLINK_PERIOD_MS 100

bool is_pid_thread_started = false; 
bool is_pid_initialized = false;

typedef struct pid_info
{
    bool pid_enabled;
    bool error;
    encoder_t encoder;
    float P;
    float I;
    float D;
    int target;
    int history_ptr;
    int max_ticks;
} pid_info;

pid_info pid_infos[6];
int last_encoder_readings[6] = {0};

/* motor locking is a way to make sure that the encoder readings of
   two motors match exactly such as in R2 where both updown motors
   must be level so the bot does not tip */
/* This is a 2d array which stores which motors are locked in the form
   of an adjacency matrix.  if motor_locking[m1][m2] == 1 then they
   are locked together. */
/* Since the locking property goes both ways, if m1 is locked to m2,
   then m2 must be locked to m1, the matrix must be symmetric */
static int motor_locking[6][6] = {0};

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

// It will always presume that negative dction is toward the limit
// switch
void calibrate(motor_t motor, limit_sw_t limit_switch)
{

    ESP_LOGI("calibration", "Starting calibration of motor %d, limit_sw: %d", motor + 1, limit_switch + 1);
    
    motor_set_speed(motor, -500);

    led_set_on(motor + 1, 0, 0, 255);
    
    while (!limit_get_pressed(limit_switch))
    {
	vTaskDelay(pdMS_TO_TICKS(10));
    }

    led_set_off(motor + 1);
    
    motor_set_speed(motor, 500);
    vTaskDelay(pdMS_TO_TICKS(75));
    motor_set_speed(motor, 0);

    ESP_LOGI("calibration", "Finished Calibration");
}

static void pid_loop()
{
    if (is_pid_thread_started)
	return;

    is_pid_thread_started = true;

    unsigned int blink_counter = 0;
    bool blink_state = true;
    
    ESP_LOGI("pid_loop", "Starting PID loop task");
    while (true)
    {
	blink_counter += 2;
	if (blink_counter % BLINK_PERIOD_MS == 0)
	    blink_state = !blink_state;
	
	for (motor_t motor = 0; motor < 6; motor++)
	{

	    /* simple PID logic */
	    pid_info info = pid_infos[motor];
	    
	    if (info.error)
	    {
		if (blink_state)
		    led_set_on(motor + 1, 255, 0, 0);
		else
		    led_set_off(motor + 1);
	    }

	    if (!info.pid_enabled)
		continue;	
	    
	    int position = encoder_get_position(info.encoder);
	    int error = info.target - position;
	    int last_error = info.target - last_encoder_readings[motor];
	    int d_error = error - last_error;

	    /* motor locking logic */
	    int motor_locking_error = 0;
	    int num_locked_motors = 0;
	    for (motor_t other_motor = 0; other_motor < 6; other_motor++)
	    {
		if (other_motor == motor)
		    continue;

		pid_info other_motor_info = pid_infos[other_motor];
		if (!other_motor_info.pid_enabled)
		    continue;

		if (!motor_locking[motor][other_motor])
		    continue;
		
		int other_motor_position = encoder_get_position(other_motor_info.encoder);
		int curr_motor_locking_error = other_motor_position - position;

		num_locked_motors += 1;
		motor_locking_error += curr_motor_locking_error;
	    }
	    
	    if (num_locked_motors == 0) /* just to avoid divide by zero */
		num_locked_motors = 1;
		    
	    int motor_locking_speed = 1.0 * ((float)motor_locking_error / (float)num_locked_motors);


	    /* ESP_LOGI("pid", */
	    /* 	     "motor: %d, motor_locking_error: %d", */
	    /* 	     motor + 1, */
	    /* 	     motor_locking_error); */

	    /* Combined speed calculation */

	    
	    /* ESP_LOGI("pid", */
	    /* 	     "motor: %d, d_err: %d", */
	    /* 	     motor, */
	    /* 	     d_error); */

	    
	    int simple_pid_speed = info.P * error + info.D * d_error;
	    int clamped_simple_speed = motor_get_clamped_speed(motor, simple_pid_speed);
	    int scaled_clamped_simple_speed = 0.9 * clamped_simple_speed;
	    int final_speed = scaled_clamped_simple_speed + motor_locking_speed;

	    
	    /* stall will only take into account the clamped speed as
	       without this, it results in us recording arbirtarily
	       large values which quickly bring the stall above the
	       threshold */
	    int stall = get_stall(motor_get_clamped_speed(motor, final_speed),
				  last_encoder_readings[motor],
				  position);

	    stall_history[motor][info.history_ptr % STALL_HISTORY_SIZE] = stall;
	    pid_infos[motor].history_ptr += 1;
	    last_encoder_readings[motor] = position;


	    
	    /* ESP_LOGI("pid_loop", "motor %d, speed: %d", */
	    /* 	     motor, */
	    /* 	     speed); */

	    int avg_stall = get_avg_stall(motor);

	    /* ESP_LOGI("stall", "avg_stall: %d", avg_stall); */

	    if (avg_stall > motor_get_max_pwm(motor))
	    {
		motor_set_speed(motor, 0);
		ESP_LOGE("motor", "Motor stall detected, stopping motor %d", motor + 1);
	    }
	    else
	    {
		motor_set_speed_smooth(motor, final_speed);
	    }
	}

	vTaskDelay(pdMS_TO_TICKS(2));
    }
}


static int clamp(int input, int lower, int upper)
{
    
    if (upper <= lower)
	{
	    ESP_LOGE("pid", "clamp upper limit %d is less equal to lower limit %d", upper, lower);
	    return 0;
	}
    
    if (input > upper)
	return upper;
    if (input < lower)
	return lower;

    return input;
}

void pid_goto(motor_t motor, int target)
{
    if (!pid_infos[motor].pid_enabled)
	return;
    
    pid_infos[motor].target = clamp(target, 0, pid_infos[motor].max_ticks);
}

void pid_init()
{

    
    if (is_pid_initialized)
	return;

    ESP_LOGI("pid", "initializing task...");
    
    is_pid_initialized = true;
    
    for (int i=0;i<6;i++)
    {
	pid_info p = {
	    .pid_enabled = false,
	    .error = false,
	    .encoder = 0,
	    .P = 0,
	    .I = 0,
	    .D = 0,
	    .target = 0,
	    .max_ticks = 0,
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

bool pid_calibrate_encoder(motor_t motor, encoder_t encoder)
{
    pid_init();

    encoder_init(encoder);

    motor_set_pwm_limit(motor, 2047);
    motor_set_speed(motor, 2047);
    

    vTaskDelay(pdMS_TO_TICKS(30));

    motor_set_pwm_limit(motor, 512);
    motor_set_speed(motor, 0);
    
    if (encoder_get_position(encoder) > 0)
    {
	encoder_set_direction(encoder, 1);
	return true;
    }
    else if (encoder_get_position(encoder) < 0)
    {
	encoder_set_direction(encoder, -1);
	return true;
    }
    else
    {
	ESP_LOGE("pid",
		 "ERROR!!! Motor M_%d, & Encoder E_%d Don't Match",
		 motor+1,
		 encoder+1);

	pid_infos[motor].error = true;
	
	return false;
    }
}

void lock_motors(motor_t m1, motor_t m2)
{
    motor_locking[m1][m2] = 1;
    motor_locking[m2][m1] = 1;
}

void separate_motors(motor_t m1, motor_t m2)
{
    motor_locking[m1][m2] = 0;
    motor_locking[m2][m1] = 0;
}

void pid_register(motor_t motor,
		  encoder_t encoder,
		  float P,
		  float D,
		  int max_ticks)
{
    if(!pid_calibrate_encoder(motor, encoder))
	return;			/* dont enable pid if calibration fails */

    pid_info info = {
	.pid_enabled = true,
	.error = false,
	.encoder = encoder,
	.P = P,
	.I = 0.0,
	.D = D,
	.target = 0,
	.max_ticks = max_ticks
    };
    
    pid_infos[motor] = info;
}

