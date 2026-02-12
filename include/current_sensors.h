#pragma once

typedef enum current_sensor_t
{
    CS1 = 0,
    CS2 = 1,
    CS3 = 2
} current_sensor_t;

void current_sensors_init(void);

#warning "All currents are in Amperes"

float sensor_get_current(current_sensor_t);
