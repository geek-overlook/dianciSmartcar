#include "headfile.h"

extern float set_turn;
char bizhangSTARTFlag = 0;
int switch_shizi_circle = 0; 			//Ԫ�ز�����ʾ�л�
extern int judge_sign;		//Ԫ���ж�������־


char motor_start = 0;
char key_function_flag = 1;
char cricle_order = 20;
char bizhang_order = 0;
char cricle_function_switch = 1;


void key(void)			//���ð�������OLED
{
	if(KEY0 == 0)      //�л�ģʽ
	{
		delay_ms(10);
		if(KEY0 == 0)
		{
			while(KEY0 == 0);
			
			if(motor_start==1)
			{
				l_pwm=0;		//�ֶ����߱���
				r_pwm=0;
				mr_pid.Bias=0;
				ml_pid.Bias=0;

				pwm_duty(MOTOR1,0); //��
				pwm_duty(MOTOR2,0); //��
				
				motor_start = 0;
				
			}
			else
			{
				motor_start = 1;
			}
		
		}
	}

	if(KEY1 == 0)
	{
		delay_ms(10);
		if(KEY1 == 0)
		{

			while(!KEY1);	//�ȴ������ɿ�
			ips_page+=1;
			if(ips_page==4)
			{
				ips_page=1;
			}
			
		}
	}
	
	
	if(KEY2 == 0)					//�����л�ҳ��
	{
		delay_ms(3);
		if(KEY2 == 0)
		{
			while(KEY2 == 0);
			
			key_function_flag+=1;
			if(key_function_flag==5)
			{
				key_function_flag=1;
			}
			
			
			
		}
	}
	
	if(KEY3 == 0)
	{
		delay_ms(10);
		if(KEY3 == 0)
		{
			while(KEY3 == 0);
			
			switch(key_function_flag)	//���ܼ�
			{
			  case 1:		//����˳��
					
					bizhang_order += 2;
					if(bizhang_order > 20)
					{
						bizhang_order = 0;
					}
					break;
				
				case 2:		//Բ��˳��
					
					cricle_order += 2;
					if(cricle_order > 20)
					{
						cricle_order = 0;
					}
					break;
				
				case 3:		//�ı��ٶ�
					
					r_target_speed += 2;
					l_target_speed += 2;
					target_speed_save = l_target_speed;
					break;
				
				case 4:
					
					cricle_function_switch++;
					if(cricle_function_switch > 1)
					{
						cricle_function_switch = 0;
					}
					
			}
				
		}
	}
	
	if(KEY4 == 0)
	{
		delay_ms(10);
		if(KEY4 == 0)
		{
			while(KEY4 == 0);	//�ȴ������ɿ�
			
		switch(key_function_flag)
			{
			  case 1:
					
					bizhang_order += -1;
					break;
				
				case 2:
					
					cricle_order += -1;
					break;
				
				case 3:
					
					r_target_speed += -1;
					l_target_speed += -1;
					target_speed_save = l_target_speed;
					break;
				
			}
		}
	}
	
	
}


