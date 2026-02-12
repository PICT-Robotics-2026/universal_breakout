#pragma once

typedef enum servo_t
{
    S1 = 0,
    S2 = 1,
    S3 = 2
} servo_t;

void servo_set_angle(servo_t,int);
void servo_set_limits(servo_t,int,int,int);

void servo_init(servo_t);
