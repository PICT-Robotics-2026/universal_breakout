#pragma once

typedef enum motor_t
{
    M1 = 0,
    M2 = 1,
    M3 = 2,
    M4 = 3,
    M5 = 4,
    M6 = 5,
} motor_t;

void motor_init(motor_t);
void motor_set_pwm_limit(motor_t,int);
void motor_set_speed(motor_t,int);
