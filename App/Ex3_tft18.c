#include <stdio.h>
#include "systick.h"
#include "Ex3.h"
#include "image_font.c"

typedef enum {
	UP,
	DOWN,
	STILL
} qesdir;

uint8_t menu_item[8][20]=
{
	"1.Welcome2SJTU",
	"2.DancingLight",
	"3.RGB-Composer",
	"4.-Gradienter-",
	"5.PlayGIF-EPRM",
	"6.PlayVideo-TF",
	"7.--RTC&UART--",
	"8.SystemInform",
};
//Ex3_1_tft18_text(): ����ʾһ����̬�˵�����
//��ɫLED�е���ɫ��0.2s��������仯һ��
void Ex3_1_tft18_text()
{
	uint8_t i=0;
  
	hsp_tft18_clear(BLACK);
	hsp_tft18_show_str(32, 0, menu_item[0]);
	hsp_tft18_show_str(32, 1, menu_item[1]);
	hsp_tft18_show_str(32, 2, menu_item[2]);
	hsp_tft18_show_str(32, 3, menu_item[3]);
	hsp_tft18_show_str(32, 4, menu_item[4]);
	hsp_tft18_show_str(32, 5, menu_item[5]);
	hsp_tft18_show_str(32, 6, menu_item[6]);
	hsp_tft18_show_str(32, 7, menu_item[7]);
	
	while(1)
	{
		i++;
		if(i > 7)
		  i = 0;
		hsp_tft18_draw_menu_cursor(12, i, WHITE);
		delay_1ms(200);
		hsp_tft18_draw_menu_cursor(12, i, BLACK);
		LED_G_TOGGLE();
	}
}
//Ex3_2_tft18_menu(): �ɵ��ڹ��Ĳ˵�����
//���ݴ������ʵ���������⹤������
uint8_t Ex3_2_tft18_menu()
{
	uint8_t i=0, ln=0;
	qesdir scroll = STILL;
  
	hsp_tft18_clear(BLACK);

	for(i=0; i<8; i++)
	{
		hsp_tft18_show_str(32, i, menu_item[i]);
	}
	
	while(1)
	{
		if (!S1())
		{
			delay_1ms(10);
			if (!S1())
				scroll = UP;
			while(!S1());
		}
		if (!S2())
		{
			delay_1ms(10);
			if (!S2())
				scroll = DOWN;
			while(!S2());
		}
        
		if (UP == scroll)    // cursor move up
		{
			if (ln>0)
			{
				 hsp_tft18_draw_menu_cursor(12, ln--, BLACK);
			}
			else
			{
				BUZZ_ON();
				delay_1ms(10);
				BUZZ_OFF();
				delay_1ms(10); 
			}
			BUZZ_ON();
			delay_1ms(10);
			BUZZ_OFF();
			hsp_tft18_draw_menu_cursor(12, ln, WHITE);
		}
		if (DOWN == scroll)    // cursor move down
		{
			if (ln<7)
			{
				hsp_tft18_draw_menu_cursor(12, ln++, BLACK);
			}
			else
			{
				BUZZ_ON();
				delay_1ms(10);
				BUZZ_OFF();
				delay_1ms(10); 
			}
			BUZZ_ON();
			delay_1ms(10);
			BUZZ_OFF();
			hsp_tft18_draw_menu_cursor(12, ln, WHITE);
		}
		scroll = STILL;

		if (!S3())    // push button pressed        
		{
			delay_1ms(50);  // de-jitter
			if (!S3())
			{
				while(!S3());
				//return ln;	// to be used in a loop
			}
		}
	}
}
//Ex3_3_bitmap(): ��ʾ�γ�����ͼƬ
void Ex3_3_bitmap()
{
	hsp_tft18_clear(BLACK);

	//hsp_tft18_draw_image(gImage_nigilent);
	//hsp_tft18_draw_image(gImage_Mario);
	hsp_tft18_draw_image(gImage_iKun2);
	//hsp_tft18_draw_image(gImage_2);
	
	while(1)
	{
		if(!S3()) break;
	}
}

void Ex3_4_bitmap2()
{
	hsp_tft18_clear(BLACK);
	hsp_tft18_draw_image(gImage_Mario);
	delay_1ms(50);
}
