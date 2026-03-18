#pragma once

#include "servos.h"

void smart_servos_init();
void smart_servo_set_angle(servo_t servo, int angle);
void smart_servo_enable(servo_t servo, float current_limit);

