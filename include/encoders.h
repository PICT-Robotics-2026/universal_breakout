#pragma once

#include <stdbool.h>

typedef enum encoder_t
{
    E1 = 0,
    E2 = 1,
    E3 = 2,
    E4 = 3
} encoder_t;

void encoder_init(encoder_t);
int encoder_get_position(encoder_t);

void encoder_set_direction(encoder_t,int);
int encoder_get_direction(encoder_t);
void encoder_print_directions();
