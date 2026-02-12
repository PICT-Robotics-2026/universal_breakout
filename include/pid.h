#pragma once

#include "encoders.h"
#include "cytrons.h"

void print_encoder_direction();
void pid_calibrate_encoder(motor_t,encoder_t);

void pid_goto(motor_t,int);
void pid_init();
void pid_register(motor_t,
		  encoder_t,
		  float);
