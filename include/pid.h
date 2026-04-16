#pragma once

#include "encoders.h"
#include "cytrons.h"
#include "limit_switches.h"

void print_encoder_direction();
void pid_calibrate_encoder(motor_t,encoder_t);

void lock_motors(motor_t,motor_t);
void separate_motors(motor_t,motor_t);
void pid_goto(motor_t,int);
void pid_init();
void pid_register(motor_t,
		  encoder_t,
		  float,
		  float,
		  int);

void calibrate(motor_t, limit_sw_t);

