#include "headfile.h"

int r_encoder =0;
int l_encoder =0;
//左右轮速度
float r_target_speed = 16.0;
float l_target_speed = 16.0;

float r_last_speed=0;
float angle_speed=0;
float turn_speed=0;

float set_turn=0.37;//？？？
float forward_flag=0.17;

//float fab_l_speed=0;
//float fab_r_speed=0;

//实际速度
float r_speed=0.0,l_speed=0.0;
//转弯计算速度
float r_endspeed=0;
float l_endspeed=0;
float l_error=0;
float r_error=0;

char turn_count=0;

float last_in_L=0;
float last_in_R=0;
float T=0.005;

float last_in_turn=0;
void motor(void)
{
	//编码器读取位移
	l_encoder = ctimer_count_read(MOTOR1_ENCODER);
	r_encoder = ctimer_count_read(MOTOR2_ENCODER);
	//读数归零
	ctimer_count_clean(CTIM0_P34);
	ctimer_count_clean(CTIM3_P04);
	
	
//	if(motor_inc < 300 &&forward_flag==0)
//	{
//		forward_flag=0;
//	}
//	else 
//	{
//		forward_flag=0.35;
//	}
	
	
//	fab_r_speed = (float)r_encoder / 1024.0f * 200.0f ;
//	fab_l_speed = (float)l_encoder / 1024.0f * 200.0f ;		//计算出1s多少转
	
	if(MOTOR1_DIR == 0)																			//计算出1s多少转以及正反转
	{
		l_speed = (float)l_encoder / 1024.0f * 200.0f ;
	}
	else
	{
		l_speed = (float)l_encoder / 1024.0f * 200.0f * -1;
	}
	if(MOTOR2_DIR == 1)																			//计算出1s多少转以及正反转
	{
		r_speed = (float)r_encoder / 1024.0f * 200.0f  ;
	}
	else
	{
		r_speed = (float)r_encoder / 1024.0f * 200.0f * -1;
	}
	motor_inc += (fabs(r_speed) + fabs(l_speed)) * 0.5f; //电机积分

	
//			turn_speed=turn_PstPID(pow_error ,&turn_pid);			//转向环
	
	
	
	
	if(turn_count<5)
	{
		turn_count++;
		
	}
	else if(turn_count>=5)
	{
		
		//multistage_pid_turn();
		turn_speed=forwardfeed_turn(turn_PstPID(error ,&turn_pid));			//转向环
//		turn_speed=turn_PstPID(error ,&turn_pid);
//	angle_speed=IncPID(gyro_x*0.01,0 ,&w_pid);		//角速度环，抑制
	
		turn_count=0;
	}
			
		
	
	
//	distance_protect=1;
//	turn_speed=0;
//	angle_speed=0;
	
			if((distance_protect)&&(angle_speed<=0)&&(turn_speed<=0))//&&(angle_speed<0
			{
				r_endspeed=r_target_speed+angle_speed -turn_speed*set_turn;
				l_endspeed=l_target_speed-angle_speed +turn_speed*(1.0-set_turn);
				//？？为什么要通过这个来选择pid 12.5   速度不同，所需要的pid也不一样
				r_error=r_endspeed-r_speed;
				l_error=l_endspeed-l_speed;
				multistage_pid();

//				r_pwm = r_pwm+IncPID(r_speed,r_endspeed,&R_pid); //加速					//速度环
//				l_pwm = l_pwm+IncPID(l_speed,l_endspeed,&L_pid); 			//-angle_speed
				r_pwm = r_pwm+forwardfeed_R(IncPID(r_speed,r_endspeed,&R_pid)); //加速					//速度环
				l_pwm = l_pwm+forwardfeed_L(IncPID(l_speed,l_endspeed,&L_pid)); 			//-angle_speed
			}
			else if((distance_protect)&&(angle_speed<=0)&&(turn_speed>0))
			{
				r_endspeed=r_target_speed+angle_speed -turn_speed*(1.0-set_turn);
				l_endspeed=l_target_speed-angle_speed +turn_speed*set_turn;
				
				r_error=r_endspeed-r_speed;
				l_error=l_endspeed-l_speed;
				multistage_pid();

				r_pwm = r_pwm+forwardfeed_R(IncPID(r_speed,r_endspeed,&R_pid)); //加速					//速度环
				l_pwm = l_pwm+forwardfeed_L(IncPID(l_speed,l_endspeed,&L_pid)); 			//-angle_speed
			}
			else if((distance_protect)&&(angle_speed>0)&&(turn_speed<=0))
			{
				r_endspeed=r_target_speed+angle_speed -turn_speed*set_turn;
				l_endspeed=l_target_speed-angle_speed +turn_speed*(1.0-set_turn);
				
				r_error=r_endspeed-r_speed;
				l_error=l_endspeed-l_speed;
				multistage_pid();
				
				r_pwm = r_pwm+forwardfeed_R(IncPID(r_speed,r_endspeed,&R_pid)); //加速					//速度环
				l_pwm = l_pwm+forwardfeed_L(IncPID(l_speed,l_endspeed,&L_pid)); 			//-angle_speed
			}
			else if((distance_protect)&&(angle_speed>0)&&(turn_speed>0))
			{
				r_endspeed=r_target_speed+angle_speed -turn_speed*(1.0-set_turn);
				l_endspeed=l_target_speed-angle_speed +turn_speed*set_turn;
				
				r_error=r_endspeed-r_speed;
				l_error=l_endspeed-l_speed;
				multistage_pid();
			
				r_pwm = r_pwm+forwardfeed_R(IncPID(r_speed,r_endspeed,&R_pid)); //加速					//速度环
				l_pwm = l_pwm+forwardfeed_L(IncPID(l_speed,l_endspeed,&L_pid)); 			//-angle_speed	
			}
	  else if(distance_protect==0&&text==1)
	  {
			  l_pwm=0;
		    r_pwm=0;
		    mr_pid.Bias=0;
				ml_pid.Bias=0;

				pwm_duty(MOTOR1,0); //左
				pwm_duty(MOTOR2,0); //右
	  }



//				r_pwm = r_pwm+IncPID(r_speed,10,&R_pid); 			//速度环			+angle_speed
//				l_pwm = l_pwm+IncPID(l_speed,10,&L_pid); 	

      	//限辐	
		if(r_pwm >= 8000)
		{		r_pwm =8000;			}
		if(r_pwm <-8000)
		{		r_pwm =-8000;			}
		if(l_pwm >= 8000)
		{		l_pwm =8000;			}
		if(l_pwm <-8000)
		{		l_pwm =-8000;			}		
		
		
		if((r_pwm>=0)&&text==1)
		{
			gpio_mode(P2_4,GPO_PP);
			MOTOR2_d=1;
			pwm_duty(MOTOR2,r_pwm); 
		}
		else if((r_pwm<0)&&text==1)
		{
//		  MOTOR2_d=0;
			gpio_mode(P2_4,GPO_PP);
			MOTOR2_d=0;
			pwm_duty(MOTOR2,-r_pwm);
		}
		if((l_pwm>=0)&&text==1)
		{
			gpio_mode(P2_6,GPO_PP);
			MOTOR1_d=1;
			pwm_duty(MOTOR1 ,l_pwm );
		}
		else if((l_pwm<0)&&text==1)
		{
//			MOTOR1_d=0;
			gpio_mode(P2_6,GPO_PP);
			MOTOR1_d=0;
			pwm_duty(MOTOR1,-l_pwm); 
		}
	
	
	
	if(distance_protect==0&&text==1)
	  {
			  l_pwm=0;
		    r_pwm=0;
		    mr_pid.Bias=0;
				ml_pid.Bias=0;

				pwm_duty(MOTOR1,0); //左
				pwm_duty(MOTOR2,0); //右


	  }
		
	
}


void multistage_pid(void)
{
	if((r_error<=1)&&(r_error>=-1))
	{		 PID_Set(&R_pid,10,1.3,1);	}
	else if((r_error>1&&r_error<=3)||(r_error<-1&&r_error>=-3))		//12
	{	   PID_Set(&R_pid,20,3.2,2);	}		
	else if((r_error>3&&r_error<=5)||(r_error<-3&&r_error>=-5))		//14
	{    PID_Set(&R_pid,42,6.6,4);	}
	else if((r_error>5&&r_error<=7)||(r_error<-5&&r_error>=-7))		//16
	{	   PID_Set(&R_pid,56,9,6);	}
//	if((r_error<=7)&&(r_error>=-7))
//		{		 PID_Set(&R_pid,56,9,6);	}
	else if((r_error>7&&r_error<=9)||(r_error<-7&&r_error>=-9))		//18
	{	   PID_Set(&R_pid,70,12.5,8);	}
	else if((r_error>9&&r_error<=11)||(r_error<-9&&r_error>=-11))		//20
	{    PID_Set(&R_pid,84,14.5,10);		}
	else if((r_error>11&&r_error<=13)||(r_error<-11&&r_error>=-13))		//22
	{	   PID_Set(&R_pid,98,17.3,12);	}
	else if((r_error>13&&r_error<=15)||(r_error<-13&&r_error>=-15))		//24
	{    PID_Set(&R_pid,112,19.7,14);	}
	else if((r_error>15&&r_error<=17)||(r_error<-15&&r_error>=-17))		//12
	{	   PID_Set(&R_pid,140,21.5,15);	}
	else if((r_error>17&&r_error<=19)||(r_error<-17&&r_error>=-19))		//14
	{    PID_Set(&R_pid,178,22.9,16);	}
	else if((r_error>19&&r_error<=21)||(r_error<-19&&r_error>=-21))		//12
	{	   PID_Set(&R_pid,185,24,17);	}
	else if((r_error>21&&r_error<=23)||(r_error<-21&&r_error>=-23))		//14
	{    PID_Set(&R_pid,195,25.5,18);	}
	else if((r_error>23&&r_error<=25)||(r_error<-23&&r_error>=-25))		//12
	{	   PID_Set(&R_pid,205,27,18);	}
	else if((r_error>25&&r_error<=28)||(r_error<-25&&r_error>=-28))		//14
	{    PID_Set(&R_pid,215,28.5,18);	}
	else if((r_error>28&&r_error<=31)||(r_error<-28&&r_error>=-31))		//14
	{    PID_Set(&R_pid,230,30,18);	}
	else if((r_error>31&&r_error<=35)||(r_error<-31&&r_error>=-35))		//14
	{    PID_Set(&R_pid,250,32,18);	}
	else if((r_error>35&&r_error<=40)||(r_error<-35&&r_error>=-40))		//14
	{    PID_Set(&R_pid,270,34,18);	}
	else if((r_error>40&&r_error<=45)||(r_error<-40&&r_error>=-45))		//14
	{    PID_Set(&R_pid,300,37,18);	}
	else if((r_error>50)||(r_error<-50))		//16
	{    PID_Set(&R_pid,340,40,18); 	}
	
	if((l_error<=1)&&(l_error>=-1))
	{		 PID_Set(&L_pid,10,1.3,1);	}
	else if((l_error>1&&l_error<=3)||(l_error<-1&&l_error>=-3))		//2
	{	   PID_Set(&L_pid,20,3.2,2);	}		
	else if((l_error>3&&l_error<=5)||(l_error<-3&&l_error>=-5))		//4
	{    PID_Set(&L_pid,42,6.6,4);	}
	else if((l_error>5&&l_error<=7)||(l_error<-5&&l_error>=-7))		//6
	{	   PID_Set(&L_pid,56,9,6);	}
//	if((l_error<=7)&&(l_error>=-7))
//		{		 PID_Set(&L_pid,56,9,6);	}
	else if((l_error>7&&l_error<=9)||(l_error<-7&&l_error>=-9))		//8
	{	   PID_Set(&L_pid,70,12.5,8);	}
	else if((l_error>9&&l_error<=11)||(l_error<-9&&l_error>=-11))		//10
	{    PID_Set(&L_pid,84,14.5,10);		}
	else if((l_error>11&&l_error<=13)||(l_error<-11&&l_error>=-13))		//12
	{	   PID_Set(&L_pid,98,17.3,12);	}
	else if((l_error>13&&l_error<=15)||(l_error<-13&&l_error>=-15))		//14
	{    PID_Set(&L_pid,112,19.7,14);	}
	else if((l_error>15&&l_error<=17)||(l_error<-15&&l_error>=-17))		//12
	{	   PID_Set(&L_pid,140,21.5,15);	}
	else if((l_error>17&&l_error<=19)||(l_error<-17&&l_error>=-19))		//14
	{    PID_Set(&L_pid,178,22.9,16);	}
	else if((l_error>19&&l_error<=21)||(l_error<-19&&l_error>=-21))		//12
	{	   PID_Set(&L_pid,185,24,17);	}
	else if((l_error>21&&l_error<=23)||(l_error<-21&&l_error>=-23))		//14
	{    PID_Set(&L_pid,195,25.5,18);	}
	else if((l_error>23&&l_error<=25)||(l_error<-23&&l_error>=-25))		//12
	{	   PID_Set(&L_pid,205,27,18);	}
	else if((l_error>25&&l_error<=28)||(l_error<-25&&l_error>=-28))		//14
	{    PID_Set(&L_pid,215,28.5,18);	}
	else if((l_error>28&&l_error<=31)||(l_error<-28&&l_error>=-31))		//14
	{    PID_Set(&L_pid,230,30,18);	}
	else if((l_error>31&&l_error<=35)||(l_error<-31&&l_error>=-35))		//14
	{    PID_Set(&L_pid,250,32,18);	}
	else if((l_error>35&&l_error<=40)||(l_error<-35&&l_error>=-40))		//14
	{    PID_Set(&L_pid,270,34,18);	}
	else if((l_error>40&&l_error<=45)||(l_error<-40&&l_error>=-45))		//14
	{    PID_Set(&L_pid,300,37,18);	}
	else if((l_error>50)||(l_error<-50))		//16
	{    PID_Set(&L_pid,340,40,18); 	}
	
	
}





void multistage_pid_turn(void)
{
//	float error_tmp = fabs(error);
//	if((error_tmp>0)&&(error_tmp<=0.2))
//	{		 PID_Set(&turn_pid,105,15,0);	}								//105,15,0		
//	else if((error_tmp>0.2)&&(error_tmp<=0.4))
//	{		 PID_Set(&turn_pid,115,10.5,0);	}
//	else if(error_tmp>0.4&&error_tmp<=0.6)		
//	{	   PID_Set(&turn_pid,117,11,0);	}		
//	else if(error_tmp>0.6&&error_tmp<=0.9)		
//	{	   PID_Set(&turn_pid,117,12,0);	}
//	else if(error_tmp>0.9)		
//	{	   PID_Set(&turn_pid,120,13.5,0);	}
}


//采样周期T
//转动惯量J=1
//摩擦系数f=1
//前馈环节：Gf(s)=s+1
//角速度/力矩：G(s)=1/(s+1)
//输出：out=(in-last_in)/T+in ;
float forwardfeed_L(float inc_in)				
{
	float inc_out;
	inc_out=(inc_in-last_in_L)*forward_flag+inc_in ;
//	out=(1.0/in)+in ;
	last_in_L = inc_in;
	return inc_out;
}

float forwardfeed_R(float inc_in)				
{
	float inc_out;
	inc_out=(inc_in-last_in_R)*forward_flag+inc_in ;
//	out=(1.0/in)+in ;
	last_in_R = inc_in;
	return inc_out;
}



/**
		滤波函数
		？？？这个系数会带来什么效果
***/
float forwardfeed_turn(float inc_in)				
{
	float inc_out;
	inc_out=(inc_in-last_in_turn)*0.15+inc_in ;
//	out=(1.0/in)+in ;
	last_in_turn = inc_in;
	return inc_out;
}




