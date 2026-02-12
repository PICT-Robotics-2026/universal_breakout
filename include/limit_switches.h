#pragma once

#warning "Limit Switches 1 and 2 must never be pressed simultaneously"
#warning "Limit Switches 3 and 4 must never be pressed simultaneously"

typedef enum limit_sw_t
{
    L1 = 0,
    L2 = 1,
    L3 = 2,
    L4 = 3
} limit_sw_t;

void limit_switches_init();
int limit_get_pressed(limit_sw_t);
