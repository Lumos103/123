#ifndef _EX6_H
#define _EX6_H

#include "hsp_timer.h"
#include "HSP_MOTOR.h"

void hsp_demo_motor_closeloop(void);
void hsp_demo_motor_openloop(void);
void hsp_demo_servo(void);
void hsp_demo_frame_servo();
int16_t motor_get_speed(void);

#endif