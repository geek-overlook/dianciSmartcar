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

void key_process(void);//按键更改layer[0],layer[1],通过adjust函数修改参数
void lcd_choice_function();//显示参数名以及数值



#endif 

