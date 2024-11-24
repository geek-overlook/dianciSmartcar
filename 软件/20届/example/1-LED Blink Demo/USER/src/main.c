/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-12-18
 ********************************************************************************************************************/

#include "headfile.h"


/*
 * 系统频率，可查看board.h中的 FOSC 宏定义修改。
 * board.h文件中FOSC的值设置为0,则程序自动设置系统频率为33.1776MHZ
 * 在board_init中,已经将P54引脚设置为复位
 * 如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可
 */

char r_speed=0.0,l_speed=0.0;
char page_buf[60];
char circle_L=0;
int time_counter = 0; //屏幕刷新
int ips_counter = 0; //屏幕清屏
int IPS_test=10;

void IPS(void);

void IPS(void)
{
	time_counter++;
	ips_counter++;
	
	if(time_counter > 5)//25ms刷新一次屏幕
	{		
		time_counter = 0;
		ips114_showint16(0,0,IPS_test);
		sprintf(page_buf,"L:%d ",r_speed);						
		if(ips_counter > 180)//刷新一次屏幕
		{
			ips_counter = 0;
			ips114_clear(WHITE);	
		}
	}

}

void main()
{
	board_init();			// 初始化寄存器,勿删除此句代码。
	lcd_init(); 
  lcd_clear(WHITE); 
	


	while(1)
	{
		
		lcd_showstr(0, 180, "RAMP_NONE ");


		delay_ms(500);
		P52 = !P52;
		

		
    }
}

