#include "cytrons.h"
#include "encoders.h"


void s_curve_register(motor_t motor, encoder_t encoder, int max_speed);
void s_curve_init();
void s_curve_goto(motor_t motor, int new_position);
