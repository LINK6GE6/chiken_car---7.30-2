#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"
#include "tim.h"

void encoder_init_start(void);
int16_t encoder_read_speed(uint8_t motor);
int16_t encoder_read_speed_10(uint8_t motor);

#endif
