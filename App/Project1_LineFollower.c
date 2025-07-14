// https://blog.csdn.net/m0_53966219/article/details/126711218
// https://blog.csdn.net/zhuoqingjoking97298/article/details/120093315
// PID: https://zhuanlan.zhihu.com/p/586532545?utm_id=0
// https://blog.csdn.net/weixin_42208428/article/details/122173575
// https://blog.csdn.net/weixin_43964993/article/details/112383192

#include "Project1.h"

extern image2_t image2_use;			// use 1/3 of the original image (40 continuous lines in the middle)
extern image2_t image2_show;		// show 1/3 of the full size screen
extern image2_t image2_temp;		// show 1/3 of the full size screen
extern uint8_t image_ready;			// MT9V034 new data frame ready in buffer
extern uint8_t image_size;				// 0: full size; 1: half size; 2: 1/3 sized
extern int16_t encoder_speed;
extern uint8_t buzz_flag;

typedef enum {
    UP,
    DOWN,
    STILL
} QES;

QES Scroll1;

float kp_pw = 3.7, ki_pw = 0, kd_pw = 12;
float kp_dc = 0.03, ki_dc = 0.001, kd_dc = 0.002;
//uint16_t fast_speed = 155, slow_speed = 120;
//uint16_t fast_speed = 120, slow_speed = 80;
uint16_t fast_speed = 90, slow_speed = 60;
uint8_t big_k = 20, small_k = 15;

uint16_t tloss = 0;               // target lost loop counter

uint32_t get_system_time_ms(void) {
    static uint32_t system_tick = 0;
    delay_1ms(1);
    system_tick++;
    return system_tick;
}
// Project#1: Line Following Robot (LFR)
void aclab_pid_demo(void)
{
	uint16_t pw = 1500, pwt = 0;
	uint16_t dc = 0;
	uint16_t tloss = 0;				// target lost loop counter
	control cur_control;
	uint8_t state_pha, state_phb;
	uint8_t state_pha_t, state_phb_t;
	    uint8_t reverse_mode = 0;          // 倒车模式标志
        uint32_t reverse_start_time = 0;   // 倒车开始时间
        uint32_t reverse_duration = 80;      // 倒车持续时间(ms)
        #define REVERSE_SPEED 100          // 倒车速度
        #define REVERSE_DISTANCE 500     // 倒车距离(mm)
        #define REVERSE_TIME 80        // 倒车时间(ms)，假设1秒行驶500mm
	uint16_t threshold=0;
	uint8_t run = RESET;
        
        uint8_t state_bg_line= 0;                       //begin line detect state
        uint8_t state_bg_line_last = 0;
        uint8_t bg_line_count = 0;
	
	image_size = 2;         // use 1/3 of the full size
	MEN_HIGH();					// enable H-bridge

	state_pha = PHA2();			state_phb = PHB2();
	state_pha_t = state_pha;	state_phb_t = state_phb;
	image_ready = RESET;
	
	hsp_tft18_clear(BLACK);
	
	para_set();		// set PID parameters
   while(!PUSH()) {}
	run = SET;
   delay_1ms(60);
	
	while(1)
	{
		if (!PUSH())			// push button pressed        
		{
			delay_1ms(100);
			if(!PUSH())
			{
				if (run == RESET && reverse_mode == 0)
				{
                                        run = SET;
                                        state_bg_line= 0;                      
                                        state_bg_line_last = 0;
                                        bg_line_count = 0;
                                        reverse_mode = 0;

				}
				else
				{
					run = RESET;
                                        reverse_mode = 0;
				}
			
			}
			while(!PUSH()) {}
			delay_1ms(50);
		}
         if(reverse_mode)
        {
            uint32_t current_time = get_system_time_ms(); // 需要实现此函数获取系统时间
            if(current_time - reverse_start_time < reverse_duration)
            {
                // 执行倒车
                hsp_motor_voltage(MOTORB, 20);
                hsp_servo_angle(SERVO3, 1500); // 倒车时保持舵机居中
            }
            else
            {
                // 倒车完成，停止所有动作
                hsp_motor_voltage(MOTORF, 0);
                hsp_motor_voltage(MOTORB, 0);
                hsp_servo_angle(SERVO3, 1500);
                reverse_mode = 0;
            }
        }
        if(bg_line_count== 2 && !reverse_mode)
        {
            // 检测到两次起始线，开始倒车
            run = RESET;
            reverse_mode = 1;
            reverse_start_time = get_system_time_ms();
            reverse_duration = REVERSE_TIME;
        }
                if(bg_line_count== 2) run = RESET;
		
		if(image_ready == SET)
		{
			//threshold = hsp_image2_threshold_otsu(image2_use);
			//threshold = hsp_image2_threshold_mean(image2_use);
			//threshold = hsp_image2_threshold_minmax(image2_use);
			//hsp_image2_show_dma(image2_use);
			//hsp_image2_show_dma(image2_show);
			//hsp_image2_binary_minmax(image2_use, image2_temp);

			//hsp_image2_binary_sobel(image2_use, image2_temp);
			//hsp_image2_erode(image2_show, image2_temp);
			//cur_control = hsp_image_judge2(image2_temp);

			//hsp_image2_binary_sobel(image2_use, image2_temp);
			//cur_control = hsp_image_judge2(image2_show);

			//threshold = hsp_image2_threshold_mean(image2_use);
			threshold = hsp_image2_threshold_otsu(image2_use);
			hsp_image2_binary(image2_use, image2_temp, threshold);
			cur_control = hsp_image_judge3(image2_temp);
                        
                        //buzz control
                        if(cross_line_judge(image2_temp)&&run == SET)
                        {
                            buzz_flag = SET;                            
                        }
                        else
                        {
                            buzz_flag = RESET;

                        }	
                        
                        state_bg_line_last = state_bg_line;
                        state_bg_line = begin_line_judge(image2_temp);
                        
                        //posedge detect
                        
                        if(state_bg_line_last == 0 && state_bg_line == 1)
                        {
                            bg_line_count++;
                              buzz_flag = SET;                            

                        }
                        //begin line detect test
                        /*
                          if(begin_line_judge(image2_temp)&&run == SET)
                        {
                            buzz_flag = SET;                            
                        }
                        else
                        {
                            buzz_flag = RESET;

                        }			
                       */ 
                        
                        
			if(!reverse_mode) // 非倒车模式下才处理循迹
            {
                pw = cur_control.pw;
                dc = cur_control.dc; 
                // apply steering angle limits
                if(1780 < pw)
                    pw = 1780;
                if(1220 > pw)
                    pw = 1220;
                // update on change
                
                if(pwt != pw)
                {
                    hsp_servo_angle(SERVO3, pw);
                    pwt = pw;
                }
                if(run == SET)
                {
                    if(dc > 0) hsp_motor_voltage(MOTORF, dc);		// run forward
                    else hsp_motor_voltage(MOTORB, -dc);		// run backward
                }
                else
                {
                    if(dc > 0) hsp_motor_voltage(MOTORF, 0);		// stop
                }
            }
			
			if(!SW1())
			{
				hsp_image2_show_dma(image2_use);
			}
			else
			{
				hsp_image2_show_dma(image2_temp);
			}
			image_ready = RESET;
		}
		
		if(!S3()) break;
	}
	
	hsp_servo_angle(SERVO1, 1500);
	hsp_servo_angle(SERVO2, 1500);
	hsp_servo_angle(SERVO3, 1500);
	hsp_servo_angle(SERVO4, 1500);
        hsp_motor_voltage(MOTORF, 0); // 确保停止电机
}

// for edge-detected image
control hsp_image_judge2(image2_t image)
{
	uint16_t pw = 1500;
	int16_t dc = 20;
	control sub_control;
	uint8_t up_black_num = 0;
	uint8_t mid_black_num = 0;
	uint8_t low_black_num = 0;
	uint8_t up_left = 255, up_right = 255, mid_left = 255, mid_right = 255, low_left = 255, low_right = 255;	// 255 is an invalid value
	uint8_t up_mid_index = 255, mid_mid_index = 255, low_mid_index = 255;	// 255 is an invalid value
	static uint8_t mid_index = 0, last_mid_index = 0;
	int16_t current_speed = 0;
	static int16_t last_speed = 0;
	int16_t target_speed = 0; 
	static int16_t last_target_speed = 0;

	up_black_num = mid_black_num = low_black_num = 0;
	up_left = up_right =mid_left = mid_right = low_left = low_right = 255;	// 255 is an invalid value
	up_mid_index = mid_mid_index = low_mid_index = 255;	// 255 is an invalid value

	// 检测上、中、下三条线的边缘线
	for (uint8_t i=1; i<IMAGEW2-1; i++)
	{
		if(image[5][i] == 0) 
		{
			up_black_num++;
			if(image[5][i-1] != 0)	// 检测到边缘
			{
				if(up_left == 255) up_left = i;
				else up_right = i;
			}
		}
		if(image[20][i] == 0) 
		{
			mid_black_num++;
			if(image[20][i-1] != 0)	// 检测到边缘
			{
				if(mid_left == 255) mid_left = i;
				else mid_right = i;
			}
		}
		if(image[35][i] == 0) 
		{
			low_black_num++;
			if(image[35][i-1] != 0)	// 检测到边缘
			{
				if(low_left == 255) low_left = i;
				else low_right = i;
			}
		}
	}
	// 显示上、中、下三条线的黑点数
//	hsp_tft18_set_region(0, 0, TFT_X_MAX-1, TFT_Y_MAX-1);
//	hsp_tft18_show_uint8(60, 1, up_black_num);
//	hsp_tft18_show_uint8(60, 2, mid_black_num);
//	hsp_tft18_show_uint8(60, 3, low_black_num);

	// 判断三条线数据的有效性
	if(up_black_num <= 25 && up_black_num >2)
	{
		if(up_right == 255)
		{
			if(up_left < 94) 
			{
				if(up_left < 10) up_mid_index = 0;
				else up_mid_index = up_left / 2;
			}
			else
			{
				if(up_left > 177) up_mid_index = 187;
				else up_mid_index = (up_left+188) / 2;
			}
		}
		else up_mid_index = (up_left + up_right) / 2;
	}
	if(mid_black_num <= 25 && mid_black_num >2)
	{
		if(mid_right == 255)
		{
			if(mid_left < 94) 
			{
				if(mid_left < 10) mid_mid_index = 0;
				else mid_mid_index = mid_left / 2;
			}
			else
			{
				if(mid_left > 177) mid_mid_index = 187;
				else mid_mid_index = (mid_left+188) / 2;
			}
		}
		else mid_mid_index = (mid_left + mid_right) / 2;
	}
	if(low_black_num <= 25 && low_black_num >2)
	{
		if(low_right == 255)
		{
			if(low_left < 94) 
			{
				if(low_left < 10) low_mid_index = 0;
				else low_mid_index = low_left / 2;
			}
			else
			{
				if(low_left > 177) low_mid_index = 187;
				else low_mid_index = (low_left+188) / 2;
			}
		}
		else low_mid_index = (low_left + low_right) / 2;
	}
	
	if(up_mid_index != 255) mid_index = up_mid_index;
	else if(mid_mid_index != 255) mid_index = mid_mid_index;
	else if(low_mid_index != 255) mid_index = low_mid_index;
	else mid_index = last_mid_index;
	sub_control.mid_index = mid_index;

//	hsp_tft18_show_uint8(0, 1, up_mid_index);
//	hsp_tft18_show_uint8(0, 2, mid_mid_index);
//	hsp_tft18_show_uint8(0, 3, low_mid_index);
//	hsp_tft18_show_uint8(0, 4, mid_index);

	int8_t cur_pw_error = 94 - mid_index;
	int8_t last_pw_error = 94 - last_mid_index;
	int8_t diff_pw_error = cur_pw_error - last_pw_error;
	pw = 1500 + kp_pw * cur_pw_error + kd_pw * diff_pw_error;
	last_mid_index = mid_index;
	sub_control.pw = pw;
	
	if(up_mid_index != 255 && low_mid_index != 255)
	{
		if(abs(up_mid_index - low_mid_index) < big_k) target_speed = fast_speed;
		else target_speed = slow_speed;
	}
	else if(up_mid_index != 255 && mid_mid_index != 255)
	{
		if(abs(up_mid_index - mid_mid_index) < small_k) target_speed = fast_speed;
		else target_speed = slow_speed;
	}
	else if(low_mid_index != 255 && mid_mid_index != 255)
	{
		if(abs(mid_mid_index - low_mid_index) < small_k) target_speed = fast_speed;
		else target_speed = slow_speed;
	}
	else target_speed = slow_speed;

	current_speed = get_speed();
	int16_t cur_speed_error = target_speed - current_speed;
	int16_t last_speed_error = last_target_speed - last_speed;
	int16_t diff_speed_error = cur_speed_error - last_speed_error;
	dc = 8 + kp_dc * cur_speed_error + kd_dc * diff_speed_error;
	if(dc > 30) dc = 30;
	if(dc < -30) 
	{
		if(target_speed == fast_speed) dc = 0;
		else dc = -30;
	}
//	hsp_tft18_show_int16(100, 1, target_speed);
//	hsp_tft18_show_int16(100, 2, current_speed);
//	hsp_tft18_show_int16(100, 3, dc);
	sub_control.dc = dc;
	
	return sub_control;
}

// for binarized image
control hsp_image_judge3(image2_t image)
{
	uint16_t pw = 1500;
	int16_t dc = 20;
	control sub_control;
	static uint8_t mid_index = 0, last_mid_index = 0;
	int16_t current_speed = 0;
	static int16_t last_speed = 0;
	int16_t target_speed = 0; 
	static int16_t last_target_speed = 0;
        static int16_t int_speed_error = 0;



	uint8_t i, j;
	uint8_t gte_l, gte_r, gte_ok;				// guide tape edge flag
	uint8_t gte_l_idx, gte_r_idx, gte_c_idx;		// guide tape index
	
	gte_l = RESET;
	gte_r = RESET;
	gte_ok = RESET;
	for(i=2; i<(IMAGEW2-2); i++)
	{
		if(RESET == gte_l)
		{
			if((255 == image[20][i]) && (0 == image[20][i+1]))	// left edge found
			{
				gte_l = SET;
				gte_l_idx = i;									// left edge index
			}
		}
		if((SET == gte_l) && (RESET == gte_r))
		{
			if((0 == image[20][i]) && (255 == image[20][i+1]))	// right edge found
			{
				gte_r = SET;
				gte_r_idx = i;									// right edge index
			}
		}
		if((SET == gte_l) && (SET == gte_r) && (RESET == gte_ok))		// both edges found
		{
			if(((gte_r_idx - gte_l_idx) > 4) && ((gte_r_idx - gte_l_idx) < 30))		// proper tape width
			{
				gte_ok = SET;
				tloss = 0;
				gte_c_idx = (gte_r_idx + gte_l_idx) >> 1;	// tape center index
			}
			else
			{
				gte_l = RESET;
				gte_r = RESET;
				gte_ok = RESET;
				tloss += 1;
			}
		}
	}
	
	if(SET == gte_ok)
		mid_index = gte_c_idx;
	else
        {
		mid_index = last_mid_index;
                //target loss protection
                tloss ++;
        
        }

	sub_control.mid_index = mid_index;

//	hsp_tft18_show_uint8(0, 1, up_mid_index);
//	hsp_tft18_show_uint8(0, 2, mid_mid_index);
//	hsp_tft18_show_uint8(0, 3, low_mid_index);
//	hsp_tft18_show_uint8(0, 4, mid_index);

	int8_t cur_pw_error = 94 - mid_index;
	int8_t last_pw_error = 94 - last_mid_index;
	int8_t diff_pw_error = cur_pw_error - last_pw_error;
	pw = 1500 + kp_pw * cur_pw_error + kd_pw * diff_pw_error;
	last_mid_index = mid_index;
	sub_control.pw = pw;
	
        if(!SW2())
        {
	target_speed = slow_speed;
        
        }
	else
        {
       target_speed = fast_speed;
        }

	current_speed = get_speed();
	int16_t cur_speed_error = target_speed - current_speed;
	int16_t last_speed_error = last_target_speed - last_speed;
	int16_t diff_speed_error = cur_speed_error - last_speed_error;
        int_speed_error += cur_speed_error;
        if(int_speed_error> 500) int_speed_error = 500;
        if(int_speed_error< -500) int_speed_error = -500;
      
	dc = dc + kp_dc * cur_speed_error +ki_dc * int_speed_error+ kd_dc * diff_speed_error;
	if(dc > 30) dc = 30;
	if(dc < -30) 
	{
		if(target_speed == fast_speed) dc = 0;
		else dc = -30;
	}
        
          //target loss protection
        if(tloss > 10)                                  
        {
            dc = 0;
            tloss = 11;
        }

//	hsp_tft18_show_int16(100, 1, target_speed);
//	hsp_tft18_show_int16(100, 2, current_speed);
//	hsp_tft18_show_int16(100, 3, dc);
	sub_control.dc = dc;
	
	return sub_control;
}

uint16_t hsp_image_judge(image2_t image)
{
	uint16_t pw;			// pulse-width control steering angle
	uint8_t i, j;
	uint8_t gte_l, gte_r, gte_ok;				// guide tape edge flag
	uint8_t gte_l_idx, gte_r_idx, gte_c_idx;		// guide tape index
	
	gte_l = RESET;
	gte_r = RESET;
	gte_ok = RESET;
	for(i=2; i<(IMAGEW2-2); i++)
	{
		if(RESET == gte_l)
		{
			if((255 == image[20][i]) && (0 == image[20][i+1]))	// left edge found
			{
				gte_l = SET;
				gte_l_idx = i;									// left edge index
			}
		}
		if((SET == gte_l) && (RESET == gte_r))
		{
			if((0 == image[20][i]) && (255 == image[20][i+1]))	// right edge found
			{
				gte_r = SET;
				gte_r_idx = i;									// right edge index
			}
		}
		if((SET == gte_l) && (SET == gte_r) && (RESET == gte_ok))		// both edges found
		{
			if(((gte_r_idx - gte_l_idx) > 4) && ((gte_r_idx - gte_l_idx) < 30))		// proper tape width
			{
				gte_ok = SET;
				gte_c_idx = (gte_r_idx + gte_l_idx) >> 1;	// tape center index
			}
			else
			{
				gte_l = RESET;
				gte_r = RESET;
				gte_ok = RESET;
			}
		}
	}
	
	if(SET == gte_ok)
		pw = 1500 + 10 * (94 - gte_c_idx);
	else
		pw = 0;
	
	return pw;
}

int16_t get_speed(void)
{
	int16_t speed = 0;

	speed = 50 * encoder_speed/234.5;
	return speed;
}

//float kp_pw = 4.5, ki_pw = 0, kd_pw = 10;
//float kp_dc = 3, ki_dc = 0, kd_dc = 0.8;
//uint16_t fast_speed = 80, slow_speed = 60;
//uint8_t big_k = 20, small_k = 15;
void para_set()
{
	char value_str[4];
	uint8_t item=0, item_t=0;
	uint8_t StatusPHA, StatusPHB;
	uint8_t jStatusPHA, jStatusPHB;
	uint8_t change=0;		// 0: no change; 1: increase; 2: decrease
  
	hsp_tft18_set_region(0, 0, TFT_X_MAX-1, TFT_Y_MAX-1);
	hsp_tft18_show_str_color(0, 0, "-Smartcar PID Demo--", BLUE, YELLOW);
	hsp_tft18_show_str(16, 1, "TargetV:");
	hsp_tft18_show_str(24, 2, "Kp: ");
	hsp_tft18_show_str(24, 3, "Ki: ");
	hsp_tft18_show_str(24, 4, "Kd: ");
	
	sprintf(value_str, "%03d", fast_speed);
	hsp_tft18_show_str(88, 1, value_str);
	
	sprintf(value_str, "%0.1f", kp_pw);
	hsp_tft18_show_str(88, 2, value_str);
	hsp_tft18_show_str(0, 2, "->");
	
	sprintf(value_str, "%0.1f", ki_pw);
	hsp_tft18_show_str(88, 3, value_str);
	
	sprintf(value_str, "%2.1f", kd_pw);
	hsp_tft18_show_str(88, 4, value_str);

	item = item_t = 1;
	StatusPHA = jStatusPHA = PHA2();
	StatusPHB = jStatusPHB = PHB2();

	while(1)
	{
		Scroll1 = STILL;
		if (!S1())
		{
			delay_1ms(2);
			if (!S1())
			{
				Scroll1 = UP;
				if(item>1)
				{
				  item--;
				  item_t = item;
				}
			}
			while(!S1());
		}
		if (!S2())
		{
			delay_1ms(2);
			if (!S2())
			{
				Scroll1 = DOWN;
				if(item<3)
				{
				  item++;
				  item_t = item;
				}
			}
			while(!S2());
		}
		if(Scroll1 != STILL)
		{
			switch(item)
			{
			case 1:
				hsp_tft18_show_str(0, 2, "->");
				hsp_tft18_show_str(0, 3, "  ");
				hsp_tft18_show_str(0, 4, "  ");
				break;
			case 2:
				hsp_tft18_show_str(0, 2, "  ");
				hsp_tft18_show_str(0, 3, "->");
				hsp_tft18_show_str(0, 4, "  ");
				break;
			case 3:
				hsp_tft18_show_str(0, 2, "  ");
				hsp_tft18_show_str(0, 3, "  ");
				hsp_tft18_show_str(0, 4, "->");
				break;
			default:
				break;
			}
		}

		StatusPHA = PHA2();   StatusPHB = PHB2();
		change = 0;
		if ((jStatusPHA!=StatusPHA) && (RESET==StatusPHA))
		{
			if (SET == StatusPHB) 		// CW to increase
			{
					change = 1;
			}
			if (RESET == StatusPHB)   // CCW to decrease
			{
					change = 2;
			}
			delay_1ms(10);
		}
		jStatusPHA = StatusPHA;
		jStatusPHB = StatusPHB;
		
		switch(item)
		{
		case 1:		// Kp
			if(change == 1)
			{
				kp_pw += 0.1;
            if(kp_pw > 9.0)
              kp_pw = 9.0;
			}
			else if(change == 2)
			{
				if(kp_pw > 2.1 )
				  kp_pw -= 0.1;
			}
			break;
		case 2:		// Ki
			if(change == 1)
			{
				if(ki_pw < 1)
				  ki_pw += 0.1;
			}
			else if(change == 2)
			{
				ki_pw -= 0.1;
            if(ki_pw < 0.0 )
               ki_pw = 0.0;
			}
			break;
		case 3:		// Kd
			if(change == 1)
			{
				if(kd_pw < 15)
				  kd_pw += 0.5;
			}
			else if(change == 2)
			{
				kd_pw -= 0.5;
            if(kd_pw < 0.0 )
               kd_pw = 0.0;
			}
			break;
		default:		// TargetV
			break;
		}
		
		if(change != 0)
		{
			sprintf(value_str, "%0.1f", kp_pw);
			hsp_tft18_show_str(88, 2, value_str);
			
			sprintf(value_str, "%0.1f", ki_pw);
			hsp_tft18_show_str(88, 3, value_str);
			
			sprintf(value_str, "%2.1f", kd_pw);
			hsp_tft18_show_str(88, 4, value_str);
		}

		if(!PUSH())
		{
			hsp_tft18_show_str(0, 2, "  ");
			hsp_tft18_show_str(0, 3, "  ");
			hsp_tft18_show_str(0, 4, "  ");
			break;
         while(!PUSH()) {}
		}
	}
}

//float kp_dc = 3, ki_dc = 0, kd_dc = 0.8;
//uint16_t fast_speed = 80, slow_speed = 60;
//uint8_t big_k = 20, small_k = 15;
void hsp_speed_setting(void)
{
	char value_str[4];
	uint8_t item=0, item_t=0;
	uint8_t StatusPHA, StatusPHB;
	uint8_t jStatusPHA, jStatusPHB;
	uint8_t change=0;		// 0: no change; 1: increase; 2: decrease
  
	hsp_tft18_clear(BLACK);
	hsp_tft18_set_region(0, 0, TFT_X_MAX-1, TFT_Y_MAX-1);
	hsp_tft18_show_str_color(0, 0, "Speed Parameter Init", RED, YELLOW);
	hsp_tft18_show_str_color(0, 1, "Set values carefully", YELLOW, BLUE);

	hsp_tft18_show_str(24, 3, "TargetVh: ");
	hsp_tft18_show_str(24, 4, "TargetVl: ");
	hsp_tft18_show_str(24, 5, "Kp: ");
	hsp_tft18_show_str(24, 6, "Ki: ");
	hsp_tft18_show_str(24, 7, "Kd: ");
	
	sprintf(value_str, "%03d", fast_speed);
	hsp_tft18_show_str(104, 3, value_str);
	hsp_tft18_show_str(0, 3, "->");
	sprintf(value_str, "%03d", slow_speed);
	hsp_tft18_show_str(104, 4, value_str);
	
	sprintf(value_str, "%0.1f", kp_dc);
	hsp_tft18_show_str(104, 5, value_str);
	
	sprintf(value_str, "%0.1f", ki_dc);
	hsp_tft18_show_str(104, 6, value_str);
	
	sprintf(value_str, "%2.1f", kd_dc);
	hsp_tft18_show_str(104, 7, value_str);

	item = item_t = 1;
	StatusPHA = jStatusPHA = PHA2();
	StatusPHB = jStatusPHB = PHB2();

	while(1)
	{
		Scroll1 = STILL;
		if (!S1())
		{
			delay_1ms(2);
			if (!S1())
			{
				Scroll1 = UP;
				if(item>1)
				{
				  item--;
				  item_t = item;
				}
			}
			while(!S1());
		}
		if (!S2())
		{
			delay_1ms(2);
			if (!S2())
			{
				Scroll1 = DOWN;
				if(item<5)
				{
				  item++;
				  item_t = item;
				}
			}
			while(!S2());
		}
		if(Scroll1 != STILL)
		{
			switch(item)
			{
			case 1:
				hsp_tft18_show_str(0, 3, "->");
				hsp_tft18_show_str(0, 4, "  ");
				hsp_tft18_show_str(0, 5, "  ");
				hsp_tft18_show_str(0, 6, "  ");
				hsp_tft18_show_str(0, 7, "  ");
				break;
			case 2:
				hsp_tft18_show_str(0, 3, "  ");
				hsp_tft18_show_str(0, 4, "->");
				hsp_tft18_show_str(0, 5, "  ");
				hsp_tft18_show_str(0, 6, "  ");
				hsp_tft18_show_str(0, 7, "  ");
				break;
			case 3:
				hsp_tft18_show_str(0, 3, "  ");
				hsp_tft18_show_str(0, 4, "  ");
				hsp_tft18_show_str(0, 5, "->");
				hsp_tft18_show_str(0, 6, "  ");
				hsp_tft18_show_str(0, 7, "  ");
				break;
			case 4:
				hsp_tft18_show_str(0, 3, "  ");
				hsp_tft18_show_str(0, 4, "  ");
				hsp_tft18_show_str(0, 5, "  ");
				hsp_tft18_show_str(0, 6, "->");
				hsp_tft18_show_str(0, 7, "  ");
				break;
			case 5:
				hsp_tft18_show_str(0, 3, "  ");
				hsp_tft18_show_str(0, 4, "  ");
				hsp_tft18_show_str(0, 5, "  ");
				hsp_tft18_show_str(0, 6, "  ");
				hsp_tft18_show_str(0, 7, "->");
				break;
			default:
				break;
			}
		}

		StatusPHA = PHA2();   StatusPHB = PHB2();
		change = 0;
		if ((jStatusPHA!=StatusPHA) && (RESET==StatusPHA))
		{
			if (SET == StatusPHB) 		// CW to increase
			{
					change = 1;
			}
			if (RESET == StatusPHB)   // CCW to decrease
			{
					change = 2;
			}
			delay_1ms(10);
		}
		jStatusPHA = StatusPHA;
		jStatusPHB = StatusPHB;
		
		switch(item)
		{
		case 1:		// fast_speed
			if(change == 1)
			{
				fast_speed += 1;
            if(fast_speed > 150)
              fast_speed = 150;
			}
			else if(change == 2)
			{
				fast_speed -= 1;
				if(fast_speed < 50)
					fast_speed = 50;
				if(slow_speed > fast_speed)
					slow_speed = fast_speed;
			}
			break;
		case 2:		// slow_speed
			if(change == 1)
			{
	 			slow_speed += 1;
				if(slow_speed > fast_speed )
				  slow_speed = fast_speed;
			}
			else if(change == 2)
			{
				slow_speed -= 1;
				if(slow_speed < 50)
				  slow_speed = 50;
			}
			break;
		case 3:		// Kp: 0 ~ 5
			if(change == 1)
			{
				kp_dc += 0.1;
				if(kp_dc > 5)
				  kp_dc = 5;
			}
			else if(change == 2)
			{
				kp_dc -= 0.1;
            if(kp_dc < 0.0 )
               kp_dc = 0.0;
			}
			break;
		case 4:		// Ki
			if(change == 1)
			{
				ki_dc += 0.1;
				if(ki_dc > 1.0)
				  ki_dc = 1.0;
			}
			else if(change == 2)
			{
				ki_dc -= 0.1;
            if(ki_dc < 0.0 )
               ki_dc = 0.0;
			}
			break;
		case 5:		// Kd
			if(change == 1)
			{
				kd_dc += 0.1;
				if(kd_dc > 2.0)
				  kd_dc = 2.0;
			}
			else if(change == 2)
			{
				kd_dc -= 0.1;
            if(kd_dc < 0.0 )
               kd_dc = 0.0;
			}
			break;
		default:		// TargetV
			break;
		}
		
		if(change != 0)
		{
			sprintf(value_str, "%03d", fast_speed);
			hsp_tft18_show_str(104, 3, value_str);
			sprintf(value_str, "%03d", slow_speed);
			hsp_tft18_show_str(104, 4, value_str);
			
			sprintf(value_str, "%0.1f", kp_dc);
			hsp_tft18_show_str(104, 5, value_str);
			
			sprintf(value_str, "%0.1f", ki_dc);
			hsp_tft18_show_str(104, 6, value_str);
			
			sprintf(value_str, "%2.1f", kd_dc);
			hsp_tft18_show_str(104, 7, value_str);
		}

		if(!PUSH())
		{
			hsp_tft18_show_str(0, 3, "  ");
			hsp_tft18_show_str(0, 4, "  ");
			hsp_tft18_show_str(0, 5, "  ");
			hsp_tft18_show_str(0, 6, "  ");
			hsp_tft18_show_str(0, 7, "  ");
			break;
         while(!PUSH()) {}
		}
	}
}


uint8_t cross_line_judge(image2_t image)
{
        uint16_t line_start1 = 0;
        uint16_t line_end1 = 0;
        uint16_t line_start2 = 0;
        uint16_t line_end2 = 0;
        uint16_t line_start3 = 0;
        uint16_t line_end3 = 0;


        uint8_t in_line = 0;
        uint16_t max_line_width = 0;
        uint16_t line_width1 = 0;
        uint16_t line_width2 = 0;
        uint16_t line_width3 = 0;

        
        // 扫描当前行
        for (uint16_t col = 2; col < IMAGEW2 - 2; col++) {
            // 检测线开始 (白->黑 跳变)
            if (!in_line && image[20][col] == 0 )
            {
                line_start1 = col ;
                in_line = 1;
            }
            // 检测线结束 (黑->白 跳变)
            else if (in_line&&(col == IMAGEW2 - 3||  image[20][col] == 255 ) ) 
            {
                line_end1 = col;
                in_line = 0;
                break;
               
            //    // 计算线宽并检查是否满足要求
            //    uint16_t line_width = line_end - line_start;
            //    if (line_width > max_line_width) {
            //       max_line_width = line_width;
            //    }
            }
        }
        
        for (uint16_t col = 2; col < IMAGEW2 - 2; col++) {
            // 检测线开始 (白->黑 跳变)
            if (!in_line && image[23][col] == 0 )
            {
                line_start2 = col ;
                in_line = 1;
            }
            // 检测线结束 (黑->白 跳变)
            else if (in_line&&(col == IMAGEW2 - 3||  image[23][col] == 255 ) ) 
            {
                line_end2 = col;
                in_line = 0;
                break;
               
            //    // 计算线宽并检查是否满足要求
            //    uint16_t line_width = line_end - line_start;
            //    if (line_width > max_line_width) {
            //       max_line_width = line_width;
            //    }
            }
        }
        line_width1 = line_end1 - line_start1;
        line_width2 = line_end2 - line_start2;
        if(line_width1 >100&&line_width1 >100)return 1;
        else return 0;
}



uint8_t begin_line_judge(image2_t image)
{
        uint8_t state  = 0;    //area state
        uint8_t edge[6] = {0}; //edge index array
//       uint8_t edge_count = 0;//edge number counter

        
        
        
        for(uint8_t col = 0; col <IMAGEW2 - 2; col ++)
        {
            uint8_t current_pixel = image[20][col];
            uint8_t next_pixel = image[20][col+1];
            
            switch(state)
            {
            case 0:
              if(current_pixel == 0)
              {
                  state ++;
                  edge[0] = col;
              }
              break;
            case 1:
              if(current_pixel == 255)
              {
                  state ++;
                  edge[1] = col;
              }
              break;
            case 2:
              if(current_pixel == 0)
              {
                  state ++;
                  edge[2] = col;
              }
              break;
            case 3:
              if(current_pixel == 255)
              {
                  state ++;
                  edge[3] = col;
              }
              break;
            case 4:
              if(current_pixel == 0)
              {
                  state ++;
                  edge[4] = col;
              }
              break;
            case 5:
              if(current_pixel == 255)
              {
                  state ++;
                  edge[5] = col;
              }
              break;
            case 6:
              if(current_pixel == 0)
              {
                  state ++;
              }
              break;
            default:
              break;
            }
            
            
            
            
            
         }
         if(state == 5) edge[5] = IMAGEW2 - 1;         
         if(state > 6 || state < 5) return 0;
         if(edge[2]-edge[1]<8 || edge[2]-edge[1]>40) return 0;
         if(edge[4]-edge[3]<8 || edge[4]-edge[3]>40) return 0;
         return 1;

}
