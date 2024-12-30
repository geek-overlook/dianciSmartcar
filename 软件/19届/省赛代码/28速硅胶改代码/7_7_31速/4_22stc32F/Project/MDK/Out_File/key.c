#include "headfile.h" // 包含头文件，确保所有必要的函数和定义可用

extern float set_turn; // 外部变量，表示转向设置
char bizhangSTARTFlag = 0; // 标志位，表示是否开始某种操作
int switch_shizi_circle = 0; // 变量，表示某种状态的指示
extern int judge_sign; // 外部变量，用于判断某种信号

char motor_start = 0; // 电机启动状态标志
char key_function_flag = 1; // 当前按键功能标志
char cricle_order = 20; // 圆形运动的指令
char bizhang_order = 0; // 直线运动的指令
char cricle_function_switch = 1; // 圆形功能开关

void key(void) // 处理按键输入
{
	if(KEY0 == 0) // 检测 KEY0 按键
	{
		delay_ms(10); // 防抖延迟
		if(KEY0 == 0) // 再次确认按键状态
		{
			while(KEY0 == 0); // 等待按键释放
			
			if(motor_start == 1) // 如果电机已启动
			{
				l_pwm = 0; // 停止左电机
				r_pwm = 0; // 停止右电机
				mr_pid.Bias = 0; // PID 控制偏差清零
				ml_pid.Bias = 0; // PID 控制偏差清零

				pwm_duty(MOTOR1, 0); // 设置电机1 PWM 为 0
				pwm_duty(MOTOR2, 0); // 设置电机2 PWM 为 0
				
				motor_start = 0; // 更新电机状态为停止
				
			}
			else // 如果电机未启动
			{
				motor_start = 1; // 启动电机
			}
		}
	}

	if(KEY1 == 0) // 检测 KEY1 按键
	{
		delay_ms(10); // 防抖延迟
		if(KEY1 == 0) // 再次确认按键状态
		{
			while(!KEY1); // 等待按键释放
			ips_page += 1; // 切换 IPS 页面
			if(ips_page == 4) // 如果页面超过 3，重置为 1
			{
				ips_page = 1;
			}
		}
	}
	
	if(KEY2 == 0) // 检测 KEY2 按键
	{
		delay_ms(3); // 防抖延迟
		if(KEY2 == 0) // 再次确认按键状态
		{
			while(KEY2 == 0); // 等待按键释放
			
			key_function_flag += 1; // 切换按键功能
			if(key_function_flag == 5) // 如果功能超过 4，重置为 1
			{
				key_function_flag = 1;
			}
		}
	}
	
	if(KEY3 == 0) // 检测 KEY3 按键
	{
		delay_ms(10); // 防抖延迟
		if(KEY3 == 0) // 再次确认按键状态
		{
			while(KEY3 == 0); // 等待按键释放
			
			switch(key_function_flag) // 根据当前功能执行不同操作
			{
				case 1: // 直线模式
					bizhang_order += 2; // 增加直线指令
					if(bizhang_order > 20) // 如果指令超过 20，重置为 0
					{
						bizhang_order = 0;
					}
					break;
				
				case 2: // 圆形模式
					cricle_order += 2; // 增加圆形指令
					if(cricle_order > 20) // 如果指令超过 20，重置为 0
					{
						cricle_order = 0;
					}
					break;
				
				case 3: // 增加速度
					r_target_speed += 2; // 增加右电机目标速度
					l_target_speed += 2; // 增加左电机目标速度
					target_speed_save = l_target_speed; // 保存左电机目标速度
					break;
				
				case 4: // 切换圆形功能
					cricle_function_switch++; // 切换状态
					if(cricle_function_switch > 1) // 如果状态超过 1，重置为 0
					{
						cricle_function_switch = 0;
					}
					break;
			}
		}
	}
	
	if(KEY4 == 0) // 检测 KEY4 按键
	{
		delay_ms(10); // 防抖延迟
		if(KEY4 == 0) // 再次确认按键状态
		{
			while(KEY4 == 0); // 等待按键释放
			
			switch(key_function_flag) // 根据当前功能执行不同操作
			{
			  case 1: // 直线模式
					bizhang_order += -1; // 减少直线指令
					break;
				
				case 2: // 圆形模式
					cricle_order += -1; // 减少圆形指令
					break;
				
				case 3: // 减少速度
					r_target_speed += -1; // 减少右电机目标速度
					l_target_speed += -1; // 减少左电机目标速度
					target_speed_save = l_target_speed; // 保存左电机目标速度
					break;
			}
		}
	}
}
