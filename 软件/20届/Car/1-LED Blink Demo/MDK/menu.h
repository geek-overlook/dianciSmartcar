#ifndef _MENU_H
#define _MENU_H

#include "headfile.h"
#include "common.h"
#include "control.h"
#include "PID.h"

extern int start_speed;
extern int front_dot_s;
extern int k_lost;
extern int speed_chose;
extern int wifi_flag;

void key_process(void);//��������layer[0],layer[1],ͨ��adjust�����޸Ĳ���
void lcd_choice_function();//��ʾ�������Լ���ֵ



#endif 

