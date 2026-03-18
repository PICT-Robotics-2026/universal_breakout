

#include "freertos/FreeRTOS.h"

#include "servos.h"
#include "current_sensors.h"

typedef struct smart_servo_info
{
    bool enabled;
    int target_angle;
    int movement_direction;
    float current_limit;
} smart_servo_info;

static smart_servo_info servo_infos[3];
static current_sensor_t servo_current_sensors[3] = { CS1, CS2, CS3 };

static bool are_smart_servos_initialized = false;

void smart_servo_loop()
{
    while (true)
    {
	for (servo_t servo = S1; servo <= S3; servo++)
	{
	    if (!servo_infos[servo].enabled)
		continue;

	    if (sensor_get_current(servo_current_sensors[servo]) > servo_infos[servo].current_limit)
	    {
		/* if current is above limit, move in opposite direction */
		servo_infos[servo].target_angle -= (3 *  servo_infos[servo].movement_direction);
		servo_set_angle(servo, servo_infos[servo].target_angle);
	    }
		
	}

	vTaskDelay(pdMS_TO_TICKS(50));
    }
    
}

void smart_servos_init()
{
    if (are_smart_servos_initialized)
	return;

    for (servo_t servo = S1; servo <= S3; servo++)
    {
	smart_servo_info info = {
	    .enabled = false,
	    .target_angle = 0,
	    .movement_direction = 1,
	    .current_limit = 0
	};

	
	servo_infos[servo] = info;
    }
    
    xTaskCreatePinnedToCore(smart_servo_loop,
			    "smart_servo_loop",
			    8192,
			    NULL,
			    5,
			    NULL,
			    1);

    are_smart_servos_initialized = true;
}

int get_movement_direction(int initial, int final)
{
    return (final - initial) / abs(final - initial);
}


void smart_servo_enable(servo_t servo, float current_limit)
{
    if (servo_infos[servo].enabled)
    {
	servo_infos[servo].current_limit = current_limit;
	return;
    }
    
    servo_infos[servo].enabled = true;
    servo_infos[servo].target_angle = 0;
    servo_infos[servo].movement_direction = 1;
    servo_infos[servo].current_limit = current_limit;

    servo_set_angle(servo, 0);
}

void smart_servo_set_angle(servo_t servo, int angle)
{
    if (!servo_infos[servo].enabled)
	return;
    
    servo_infos[servo].movement_direction = get_movement_direction(angle, servo_infos[servo].target_angle);
    servo_infos[servo].target_angle = angle;
    servo_set_angle(servo, angle);
}
