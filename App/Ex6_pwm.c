// Exercise #6: timer related peripherals
// https://zhuanlan.zhihu.com/p/572276498?utm_id=0

#include <stdio.h>
#include <string.h>
#include "systick.h"
#include "HSP_MOTOR.h"
#include "Ex3.h"
#include "Ex6.h"

extern int16_t encoder_speed;

#define PW_MAX	1700U
#define PW_MID	1500U
#define PW_MIN	1300U

int16_t current_speed = 0;
int16_t last_speed = 0;
int16_t target_speed = 0; 
int16_t last_target_speed = 0;
int16_t cur_speed_error, last_speed_error, diff_speed_error;

/*void hsp_demo_motor_closeloop(void)
{
	uint8_t dc = 0;	
	uint16_t cmd=0, tcmd=0;
	uint8_t state_pha, state_phb;
	uint8_t state_pha_t, state_phb_t;

	uint16_t adc_data;
	char str[20];
	float vbat;
	
	int16_t err, terr, derr;
	int16_t vp, vi, vd, vsum;
	float kp = 0.05;
	float ki = 0.001;
	float kd = 0.001;

	hsp_tft18_clear(BLACK);
	MEN_HIGH();			// enable H-bridge

	state_pha = PHA2();			state_phb = PHB2();
	state_pha_t = state_pha;	state_phb_t = state_phb;
	
	hsp_tft18_show_str(0, 0, "Vbat: ");			// Battery voltage
	hsp_tft18_show_str(0, 1, "Vmot: ");			// motor voltage
	
	hsp_tft18_show_str(0, 3, "Vset: ");			// dc command ppr/20ms
	hsp_tft18_show_str(0, 4, "Fdbk: ");			// speed feedback ppr/20ms

	hsp_tft18_show_str(0, 6, "Duty Cycle: ");	// dc calculated
	//hsp_tft18_show_str(0, 7, "Status:");			// stop or run
	
	err = terr = encoder_speed;
	cur_speed_error = 0;
	last_speed_error = 0;
	diff_speed_error = 0;
	dc = 0;

	last_speed = 0;
	last_speed_error = 0;
	
	while(1)
	{
		if (!PUSH())			// push button pressed        
		{
			delay_1ms(50);		// de-jitter
			if (!PUSH())
			{
				while(!PUSH());
				target_speed = 0;
			}
		}
		
		if(target_speed == 0)
		{
			cur_speed_error = 0;
			last_speed_error = 0;
			diff_speed_error = 0;
			dc = 0;
			last_speed = 0;
			last_speed_error = 0;
		}
	
		state_pha = PHA2();			state_phb = PHB2();
		if((state_pha_t != state_pha) || (state_phb_t != state_phb))
		{
			if(state_phb_t == state_phb)
			{
				if(SET == state_phb)
				{
					if(RESET == state_pha) target_speed += 10;
					else if(10 < target_speed) target_speed -= 10;
				}
				else
				{
					if(SET == state_pha) target_speed += 10;
					else if(0 < target_speed) target_speed -= 10;
				}
			}
			else
			{
				if(SET == state_pha)
				{
					if(SET == state_phb) target_speed += 10;
					else if(0 < target_speed) target_speed -= 10;
				}
				else
				{
					if(RESET == state_pha) target_speed += 10;
					else if(0 < target_speed) target_speed -= 10;
				}
			}
			state_pha_t = state_pha;
			state_phb_t = state_phb;
		}
		
		if(200 < target_speed)
			target_speed = 200;
		current_speed = motor_get_speed();
		if(current_speed < 0) current_speed = -current_speed;
		
		hsp_tft18_show_int16(56, 3, target_speed);	// speed command in cm/s
		hsp_tft18_show_int16(56, 4, current_speed);	// speed feedback
		
		// PWM output stage, subjected to duty cycle limits
		// don't run motor at high speed for safety
		cur_speed_error = target_speed - current_speed;
		last_speed_error = target_speed - last_speed;
		diff_speed_error = cur_speed_error - last_speed_error;
		//dc = 15 + kp_dc * cur_speed_error + kd_dc * diff_speed_error;
		dc = dc + 0.02 * cur_speed_error + 0.02 * diff_speed_error;
		last_speed = current_speed;
		//last_speed_error = cur_speed_error;
		
		if(dc > 60) dc = 60;
		if(dc < 0) dc = 0;
		hsp_motor_voltage(MOTORF, dc);		// run forward

		hsp_tft18_show_int16(96, 6, dc);				// dc command

		adc_software_trigger_enable(ADC0, ADC_INSERTED_CHANNEL);
		/* wait for EOC */
/*		while(!adc_flag_get(ADC0, ADC_FLAG_EOC));
		adc_data = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_0);
		adc_flag_clear(ADC0, ADC_FLAG_EOC);		/* flag cleared automatically by data reading? */

/*		vbat	= (adc_data+1)*5.016*3.3/4096;		// in unit of Volt
		sprintf(str, "%0.3fV", vbat);
		hsp_tft18_show_str(56, 0, str);
		sprintf(str, "%0.3fV", vbat*dc/100);
		hsp_tft18_show_str(56, 1, str);
		
		if(!S3()) break;
	}

	MEN_LOW();		// disable H-bridge output
	while(!S3()) {}
} */
void hsp_demo_motor_closeloop(void)
{
    uint8_t dc = 0;    
    uint16_t cmd=0, tcmd=0;
    uint8_t state_pha, state_phb;
    uint8_t state_pha_t, state_phb_t;

    uint16_t adc_data;
    char str[20];
    float vbat;
    
    int16_t err, terr, derr;
    int16_t vp, vi, vd, vsum;
    float kp = 0.02;
    float ki = 0.001;  // ����ϵ��
    float kd = 0.002;

    // ������������ر���
    int16_t integral_error = 0;     // ��������ۼ�ֵ
    int16_t integral_error_max = 500; // �������޷�����ֹ���ֱ���
    
    hsp_tft18_clear(BLACK);
    MEN_HIGH();           // enable H-bridge

    state_pha = PHA2();            state_phb = PHB2();
    state_pha_t = state_pha;    state_phb_t = state_phb;
    
    hsp_tft18_show_str(0, 0, "Vbat: ");            // Battery voltage
    hsp_tft18_show_str(0, 1, "Vmot: ");            // motor voltage
    
    hsp_tft18_show_str(0, 3, "Vset: ");            // dc command ppr/20ms
    hsp_tft18_show_str(0, 4, "Fdbk: ");            // speed feedback ppr/20ms

    hsp_tft18_show_str(0, 6, "Duty Cycle: ");    // dc calculated
    //hsp_tft18_show_str(0, 7, "Status:");            // stop or run
    
    err = terr = encoder_speed;
    cur_speed_error = 0;
    last_speed_error = 0;
    diff_speed_error = 0;
    dc = 0;

    last_speed = 0;
    last_speed_error = 0;
    
    while(1)
    {
        if (!PUSH())            // push button pressed        
        {
            delay_1ms(50);        // de-jitter
            if (!PUSH())
            {
                while(!PUSH());
                target_speed = 0;
            }
        }
        
        if(target_speed == 0)
        {
            cur_speed_error = 0;
            last_speed_error = 0;
            diff_speed_error = 0;
            dc = 0;
            last_speed = 0;
            last_speed_error = 0;
            integral_error = 0;  // Ŀ���ٶ�Ϊ0ʱ�������
        }
    
        state_pha = PHA2();            state_phb = PHB2();
        if((state_pha_t != state_pha) || (state_phb_t != state_phb))
        {
            if(state_phb_t == state_phb)
            {
                if(SET == state_phb)
                {
                    if(RESET == state_pha) target_speed += 10;
                    else if(10 < target_speed) target_speed -= 10;
                }
                else
                {
                    if(SET == state_pha) target_speed += 10;
                    else if(0 < target_speed) target_speed -= 10;
                }
            }
            else
            {
                if(SET == state_pha)
                {
                    if(SET == state_phb) target_speed += 10;
                    else if(0 < target_speed) target_speed -= 10;
                }
                else
                {
                    if(RESET == state_pha) target_speed += 10;
                    else if(0 < target_speed) target_speed -= 10;
                }
            }
            state_pha_t = state_pha;
            state_phb_t = state_phb;
        }
        
        if(200 < target_speed)
            target_speed = 200;
        current_speed = motor_get_speed();
        if(current_speed < 0) current_speed = -current_speed;
        
        hsp_tft18_show_int16(56, 3, target_speed);    // speed command in cm/s
        hsp_tft18_show_int16(56, 4, current_speed);    // speed feedback
        
        // PID���Ƽ���
        cur_speed_error = target_speed - current_speed;
        last_speed_error = target_speed - last_speed;
        diff_speed_error = cur_speed_error - last_speed_error;
        
        // ��������㼰�޷�
        integral_error += cur_speed_error;
        if (integral_error > integral_error_max) integral_error = integral_error_max;
        if (integral_error < -integral_error_max) integral_error = -integral_error_max;
        
        // PID�������
        vp = kp * cur_speed_error;
        vi = ki * integral_error;
        vd = kd * diff_speed_error;
        vsum = vp + vi + vd;
        
        // ����ռ�ձ�
        dc = dc + vsum;
        last_speed = current_speed;
        
        // ռ�ձ��޷�
        if(dc > 60) dc = 60;
        if(dc < 0) dc = 0;
        hsp_motor_voltage(MOTORF, dc);        // run forward

        hsp_tft18_show_int16(96, 6, dc);                // dc command

        adc_software_trigger_enable(ADC0, ADC_INSERTED_CHANNEL);
        /* wait for EOC */
        while(!adc_flag_get(ADC0, ADC_FLAG_EOC));
        adc_data = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_0);
        adc_flag_clear(ADC0, ADC_FLAG_EOC);        /* flag cleared automatically by data reading? */

        vbat    = (adc_data+1)*5.016*3.3/4096;        // in unit of Volt
        sprintf(str, "%0.3fV", vbat);
        hsp_tft18_show_str(56, 0, str);
        sprintf(str, "%0.3fV", vbat*dc/100);
        hsp_tft18_show_str(56, 1, str);
        
        if(!S3()) break;
    }

    MEN_LOW();        // disable H-bridge output
    while(!S3()) {}
}

void hsp_demo_motor_openloop(void)
{
	uint16_t dc=0, tdc=0;
	uint8_t state_pha, state_phb;
	uint8_t state_pha_t, state_phb_t;
	uint8_t dir=0;		// 1: forward; 0: backward
	uint8_t tdir;
	uint16_t adc_data;
	char str[20];
	float vbat;

	hsp_tft18_clear(BLACK);
	MEN_HIGH();			// enable H-bridge

	state_pha = PHA2();			state_phb = PHB2();
	state_pha_t = state_pha;	state_phb_t = state_phb;
	
	hsp_tft18_show_str(0, 0, "Duty Cycle: ");	// dc command
	hsp_tft18_show_str(0, 2, "Pulse/20ms: ");	// speed feedback
	hsp_tft18_show_str(0, 4, "Vbat: ");			// Battery voltage
	hsp_tft18_show_str(0, 6, "Vmot: ");			// motor voltage
	
	tdir = dir = SW1();		// 1: forward; 0: backward
	dc = 0;
	
	while(1)
	{
	  	dir = SW1();
		if(tdir != dir)		// stop motor before run in reversed direction
		{
			dc = 0;
			tdir = dir;
		}
		
		if (!PUSH())			// push button pressed        
		{
			delay_1ms(50);		// de-jitter
			if (!PUSH())
			{
				while(!PUSH());
				dc = 0;
			}
		}
		
		state_pha = PHA2();			state_phb = PHB2();
		if((state_pha_t != state_pha) || (state_phb_t != state_phb))
		{
			if(state_phb_t == state_phb)
			{
				if(SET == state_phb)
				{
					if(RESET == state_pha) dc++;
					else if(0 < dc) dc--;
				}
				else
				{
					if(SET == state_pha) dc++;
					else if(0 < dc) dc--;
				}
			}
			else
			{
				if(SET == state_pha)
				{
					if(SET == state_phb) dc++;
					else if(0 < dc) dc--;
				}
				else
				{
					if(RESET == state_pha) dc++;
					else if(0 < dc) dc--;
				}
			}
			state_pha_t = state_pha;
			state_phb_t = state_phb;
		}
		
		// PWM output stage, subjected to duty cycle limits
		// don't run motor at high speed for safety
		if(60 < dc)
			dc = 60;
		if(tdc != dc)
		{
			if(dir)
			{
				hsp_motor_voltage(MOTORF, dc);		// run forward
			}
			else
			{
				hsp_motor_voltage(MOTORB, dc);		// run backward
			}
			tdc = dc;
		}
		hsp_tft18_show_int16(96, 0, dc);				// dc command
		hsp_tft18_show_int16(96, 2, encoder_speed);	// speed feedback

		adc_software_trigger_enable(ADC0, ADC_INSERTED_CHANNEL);
		/* wait for EOC */
		while(!adc_flag_get(ADC0, ADC_FLAG_EOC));
		adc_data = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_0);
		adc_flag_clear(ADC0, ADC_FLAG_EOC);		/* flag cleared automatically by data reading? */

		vbat	= (adc_data+1)*5.016*3.3/4096;		// in unit of Volt
		sprintf(str, "%0.3fV", vbat);
		hsp_tft18_show_str(56, 4, str);
		sprintf(str, "%0.3fV", vbat*dc/100);
		hsp_tft18_show_str(56, 6, str);
		
		if(!S3()) break;
	}

	MEN_LOW();		// disable H-bridge output
	while(!S3()) {}
}

void hsp_demo_servo(void)
{
	uint16_t pw, tpw;
	uint8_t mode, tmode;
	uint8_t pos_hsp, pos_standard;
	uint8_t tpos_hsp, tpos_standard;
	uint8_t state_pha, state_phb;
	uint8_t state_pha_t, state_phb_t;
	uint16_t adc_data;
	char str[20];
	float vbat;
	
	hsp_tft18_clear(BLACK);
	hsp_demo_frame_servo();

	pw = tpw = PW_MID;
	state_pha = PHA2();			state_phb = PHB2();
	state_pha_t = state_pha;	state_phb_t = state_phb;

	mode = tmode = 1;
	hsp_tft18_show_str_color(0, 7, "Limit: ON", BLUE, GREEN);
	sprintf(str, "Cmd:%04dus", pw);
	hsp_tft18_show_str_color(81, 7, str, BLACK, YELLOW);
	
	while(1)
	{
	  	mode = SW4();			// 0: limit off; 1: limit on
		if(tmode != mode)
		{
		  	if(!mode)
				hsp_tft18_show_str_color(0, 7, "Limit:OFF", BLUE, RED);
			else
				hsp_tft18_show_str_color(0, 7, "Limit: ON", BLUE, GREEN);
			tmode = mode;
		}
		
		if(!PUSH())			// push button pressed        
		{
			delay_1ms(50);		// de-jitter
			if (!PUSH())
			{
				 while(!PUSH());
				 pw = PW_MID;
			}
		}
		
		state_pha = PHA2();			state_phb = PHB2();
		if(state_pha_t != state_pha)
		{
			if(SET == state_pha)
			{
				if(RESET == state_phb) pw++;
				else pw--;
			}
			else
			{
				if(SET == state_phb) pw++;
				else pw--;
			}
			state_pha_t = state_pha;
			state_phb_t = state_phb;
			//delay_1ms(10);		// de-jitter
		}
		
		// PWM output stage, subjected to car-by-car steering angle limits
		if(mode)		// Limit on
		{
			if(PW_MAX < pw)
				pw = PW_MAX;
			if(PW_MIN > pw)
				pw = PW_MIN;
		}
		else			// Limit off
		{
			if(2500 < pw)
				pw = 2500;
			if(500 > pw)
				pw = 500;
		}
		
		if(tpw != pw)
		{
			hsp_servo_angle(SERVO3, pw);

			// draw position indicators
			pos_standard = (2500-pw)*158/2000 + 1;
			if(pw >= PW_MID)
			{
			  	//pos_hsp = 80 - (pw-PW_MID)*79/(PW_MAX-PW_MID);
			  	pos_hsp = (pw-PW_MID)*79/(PW_MAX-PW_MID);
				if(pos_hsp >= 79) pos_hsp = 1;
				else pos_hsp = 80 - pos_hsp;
			}
			else
			{
//				pos_hsp = (pw-PW_MIN)*79/(PW_MID-PW_MIN) + 1;
				pos_hsp = 80 + (PW_MID-pw)*79/(PW_MID-PW_MIN);
				if(pos_hsp > 158) pos_hsp = 158;
			}
			hsp_tft18_draw_line_v(tpos_standard, 18, 13, BLACK);
			hsp_tft18_draw_line_v(pos_standard, 18, 13, GOLD);
			hsp_tft18_draw_line_v(tpos_hsp, 51, 13, BLACK);
			hsp_tft18_draw_line_v(pos_hsp, 51, 13, GOLD);
			tpos_hsp = pos_hsp;
			tpos_standard = pos_standard;

			if(pw > PW_MAX)
			{
				hsp_tft18_draw_block(0, 81, 72, 13, RED);
			}
			else
			{
				hsp_tft18_draw_block(0, 81, 72, 13, GREEN);
			}
			if(pw < PW_MIN)
			{
				hsp_tft18_draw_block(88, 81, 72, 13, RED);
			}
			else
			{
				hsp_tft18_draw_block(88, 81, 72, 13, GREEN);
			}

			sprintf(str, "Cmd:%04dus", pw);
			hsp_tft18_show_str_color(81, 7, str, BLACK, YELLOW);
			
			tpw = pw;
		}

		adc_software_trigger_enable(ADC0, ADC_INSERTED_CHANNEL);
		/* wait for EOC */
		while(!adc_flag_get(ADC0, ADC_FLAG_EOC));
		adc_data = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_0);
		adc_flag_clear(ADC0, ADC_FLAG_EOC);		/* flag cleared automatically by data reading? */

		vbat	= (adc_data+1)*5.016*3.3/4096;		// in unit of Volt
		sprintf(str, "%0.2fV", vbat);
		hsp_tft18_show_str_color(32, 6, str, BLACK, WHITE);
		
		if(!S3()) break;
	}

	hsp_servo_angle(SERVO1, 1500);
	hsp_servo_angle(SERVO2, 1500);
	hsp_servo_angle(SERVO3, 1500);
	hsp_servo_angle(SERVO4, 1500);
}

void hsp_demo_frame_servo(void)
{
	char line[20];
	
	hsp_tft18_show_str_color(0, 0, "   STANDARD SERVO   ", BLUE, GREEN);
	hsp_tft18_show_str_color(0, 2, "   Steering Angle   ", YELLOW, BLUE);
	hsp_tft18_show_str_color(0, 6, "Bat:", BLACK, WHITE);

	// frame for standard servo
	hsp_tft18_draw_frame(0, 17, 159, 14, GREEN);
	// frame for HSP-car steering angle
	hsp_tft18_draw_frame(0, 50, 159, 14, BLUE);
//	hsp_tft18_draw_line_v(0, 18, 13, BLACK);
//	hsp_tft18_draw_line_v(159, 18, 13, BLACK);
	// block for LimitLeft sign
	hsp_tft18_draw_block(0, 81, 72, 13, GREEN);
	// block for Limitright sign
	hsp_tft18_draw_block(88, 81, 72, 13, GREEN);
	// vertical decorating seperator
	hsp_tft18_draw_block(81, 66, 7, 29, GRAY2);

	// initialize servo angle indicator
	hsp_tft18_draw_line_v(80, 18, 13, GOLD);
	hsp_tft18_draw_line_v(80, 51, 13, GOLD);

	sprintf(line, "LmtR:%04d", PW_MIN);
	hsp_tft18_show_str_color(88, 4, line, RED, BLUE);
	sprintf(line, "LmtL:%04d", PW_MAX);
	hsp_tft18_show_str_color(0, 4, line, RED, BLUE);
	sprintf(line, "Mid:%04dus", PW_MID);
	hsp_tft18_show_str_color(81, 6, line, RED, BLUE);
}

int16_t motor_get_speed(void)
{
	int16_t speed = 0;
        
	//speed = encoder_speed;

	speed = 100 * encoder_speed/2345;
	return speed;
}

