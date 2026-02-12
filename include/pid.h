#pragma once

#include "encoders.h"
#include "cytrons.h"

void print_encoder_direction();
void pid_calibrate_encoder(motor_t,encoder_t);

void pid_set_target(motor_t,int);
void pid_init();
void pid_register_motor_encoder(motor_t,
				encoder_t,
				float);
