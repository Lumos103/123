#ifndef _PROJECT1_H
#define _PROJECT1_H

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "systick.h"
#include "hsp_timer.h"
#include "Ex3.h"
#include "Ex6.h"
#include "Ex7.h"
#include "00_MENU.h"

typedef struct	\
{
	uint16_t pw;
	int16_t dc;
	uint8_t mid_index;
} control;

void aclab_pid_demo(void);
uint16_t hsp_image_judge(image2_t image);
control hsp_image_judge2(image2_t image);
control hsp_image_judge3(image2_t image);
int16_t get_speed(void);
void para_set(void);
void hsp_speed_setting(void);
uint8_t cross_line_judge(image2_t image);
uint8_t begin_line_judge(image2_t image);



#endif