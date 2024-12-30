#include "headfile.h"

int collection_shizi[10] = {1,1,1,1,1,1,1,1,1,1};
int collection_shizi_R[10] = {1,1,1,1,1,1,1,1,1,1};


//左右圆环标志位
char flag_circle=0;

//圆环判断状态量
char circle_R=0;
char circle_L=0;

//判断圆环十字的数量
char circle_count=0;
int shizi_count=0;


//十字判断标志位
char flag_shizi=0;
char flag_shizi_R=0;

//避障标志位
char barrier=0;			//检测是否避障
char flag_barrier=0;	
char barrier_count=0;
char bizhang=0;



	//电感变化趋势
char adc_state[11]={0};
float adc_cons[11]={0};

float straight_inc=0;			//直道积分
int circle_inc=0;			//环岛编码器积分
int cir_inc=0;
float motor_inc=0;			//电机积分
float bizhang_inc =0;		//避障积分

int bizhang_window[5] = {1500,1500,1500,1500,1500};	//避障滑动窗口滤波

float s_r = 3000;
float s_r_RYUAN = 3000;

float s_r_SHIZI = 3000;
float s_r_RSHIZI = 3000;
float s_r_WSHIZI = 3000;	//上十字
float s_r_LSHIZI = 3000;	//左十字

//左右圆环判断阈值
int  HUANDAO_Ryuzhi=210;
int  HUANDAO_Lyuzhi=210;

//十字判断阈值
int SHIZI_yuzhi=0;	
int SHIZI_Ryuzhi=0;

char flag_SHIZI_KNN = 0;

char flag_zhijiao = 0;
char zhijiao_L = 0;
char zhijiao_R = 0;
//void SHIZI();

char SHIZI(void);

char cricle_count_order = 0;
float target_speed_save = 0;
												 
void judge(void)
{
	
	flag_circle=YUANHUAN();				//判断左右圆环
	
	//---------------------------------------------------------------------		左圆环代码-------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
	if(flag_circle==1 && adc[3]>0.65 &&circle_R==0&&circle_L==0&&flag_shizi==0&&flag_shizi_R==0&&flag_barrier==0 && fabs(error) < 1 && (((cricle_order >> cricle_count_order) & 1)==1|| cricle_order == 20))		//判断左入环
	{
		circle_L=1;//1
		
		buzzer=0;	//蜂鸣器开
		
		straight_inc=motor_inc;//straight_inc只有在圆环时才会被赋值
	}
	
	//circle_L==1circle_L==1circle_L==1circle_L==1circle_L==1circle_L==1circle_L==1circle_L==1circle_L==1circle_L==1circle_L==1
	if(circle_L==1)//1
	{
		//根据cricle_function_switch的值，选择不同的控制逻辑
	 if(cricle_function_switch == 0)
	 {
		if(motor_inc-straight_inc<=1250)//如果motor_inc - straight_inc 超过 1250 ，电机的实际位置与期望位置或先前位置之间的差异超过了某个预定的阈值。这个阈值我认为是圆环的长度
		{
//			error=0.1;
			x_inc+=gyro_x*0.005;
			if(x_inc<cir_inc)	
			{
				cir_inc=x_inc;	
			}
//			circle_inc=(int)fabs(x_inc);
			circle_inc=(int)fabs(cir_inc);
		}
		
		
		else if(motor_inc-straight_inc>1250)
		{
			
			r_target_speed = 31.0;
			l_target_speed = 31.0;
			x_inc+=gyro_x*0.005;
			if(x_inc<cir_inc)	
			{
				cir_inc=x_inc;	
			}
//			circle_inc=(int)fabs(x_inc);
			circle_inc=(int)fabs(cir_inc);
			
			buzzer=1;			//进右圆环蜂鸣器关
			
			if(circle_inc<=350)
			{
//				error=0.65;
				error=-0.85;
				
			}
			else
			{
				
				buzzer = 0;		//出右圆环蜂鸣器开
				r_target_speed = target_speed_save;
				l_target_speed = target_speed_save;
				
				circle_L = 2;
				x_inc = 0;
				cir_inc=0;
				circle_inc = 0;
				straight_inc = motor_inc;
				
			}
		}
	 }
		
	 
	 
	 //根据cricle_function_switch的值，选择不同的控制逻辑
	 else if(cricle_function_switch)
	 {
		flag_shizi = 0;		//保证圆环内十字不判断
		
		if(motor_inc-straight_inc<=500)
		{
			error=-0.05;
			
		}
		else if(motor_inc-straight_inc>500)
	 {
			x_inc+=gyro_x*0.005;
			circle_inc=(int)fabs(cir_inc);
			
			buzzer=1;		//蜂鸣器关
			if(x_inc<cir_inc)	
			{
				cir_inc=x_inc;	
			}
//			circle_inc=(int)fabs(x_inc);
			
			
			if(circle_inc<=25)
			{
				buzzer=0;
				error=-0.35;
			}
			else if(circle_inc>25&&circle_inc<=345)
			{
				buzzer=1;
				//error=(1.25*(sqrt(adc[0])-sqrt(adc[6]))+1.25*(sqrt(adc[1])-sqrt(adc[5])))/(adc[1]+adc[5]+adc[6]+adc[0]+1.35*adc[3]);
			}
//			else if(circle_inc>320&&circle_inc<=350)
//			{
//				error=-0.35;
//				
//			}
			else
			{
				buzzer = 0;
			
//				buzzer = 0;		//出左圆环蜂鸣器开
				
				circle_L = 2;
				x_inc = 0;
				cir_inc=0;
				circle_inc = 0;
				straight_inc = motor_inc;
				
			}
		}
	 }
		
		}

	//circle_L==2circle_L==2circle_L==2circle_L==2circle_L==2circle_L==2circle_L==2circle_L==2circle_L==2circle_L==2circle_L==2
	if(circle_L==2)
	{
		
		if(motor_inc-straight_inc<1400)
		{
			error=0;
//			error += 0.04;//
			flag_shizi = 0;		//保证圆环内十字不判断
//			return;
		}
		else if(motor_inc-straight_inc>=1000&&motor_inc-straight_inc<2800)
		{
				
		}
		else
		{
			
			buzzer = 1;		//出左圆环蜂鸣器关
			circle_count++;
			cricle_count_order++;
			circle_L=0;
			straight_inc=motor_inc;
		}
		
	}
	
//---------------------------------------------------------------------		右圆环代码-------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
	
	if(flag_circle == 2 && adc[3]>0.65&& circle_R == 0 && circle_L == 0 &&flag_shizi==0&&flag_shizi_R==0&& flag_barrier==0 && fabs(error) < 1 && (((cricle_order >> cricle_count_order) & 1)==0|| cricle_order == 20))		//判断右入环
	{
		circle_R=1;
		
		buzzer=0;		//进右圆环蜂鸣器开
		
		straight_inc=motor_inc;
	}
	
	if(circle_R == 1)			//判断右入环
	{
	
		flag_shizi = 0;		//保证圆环内十字不判断
		
	if(cricle_function_switch == 0)
	{
		if(motor_inc-straight_inc<=1250)
		{
//			error=0.1;
			x_inc+=gyro_x*0.005;
			if(x_inc<cir_inc)	
			{
				cir_inc=x_inc;	
			}
//			circle_inc=(int)fabs(x_inc);
			circle_inc=(int)fabs(cir_inc);
		}
		else if(motor_inc-straight_inc>1250)
		{
			r_target_speed = 31.0;
			l_target_speed = 31.0;
			x_inc+=gyro_x*0.005;
			if(x_inc>cir_inc)	
			{
				cir_inc=x_inc;	
			}
//			circle_inc=(int)fabs(x_inc);
			circle_inc=(int)fabs(cir_inc);
			
			buzzer=1;			//进右圆环蜂鸣器关
			
			if(circle_inc<=350)
			{
//				error=0.65;
				error=0.85;
				
			}
			else
			{
				
				buzzer = 0;		//出右圆环蜂鸣器开
				r_target_speed = target_speed_save;
				l_target_speed = target_speed_save;
				
				circle_R = 2;
				x_inc = 0;
				cir_inc=0;
				circle_inc = 0;
				straight_inc = motor_inc;
				
			}
		}
	 }
		
		else if(cricle_function_switch)
	 {
		 
		if(motor_inc-straight_inc<=500)
		{
			error=0.1;
			
		}
		else if(motor_inc-straight_inc>500)
		{
			x_inc+=gyro_x*0.005;
			if(x_inc>cir_inc)	
			{
				cir_inc=x_inc;	
			}
//			circle_inc=(int)fabs(x_inc);
			circle_inc=(int)fabs(cir_inc);
			
			buzzer=1;			//进右圆环蜂鸣器关
			
			if(circle_inc<=23)
			{
				error=0.45;
			}
			else if(circle_inc>23&&circle_inc<=355)
			{
//				error=0.9;
				//error=(1.25*(sqrt(adc[0])-sqrt(adc[6]))+1.25*(sqrt(adc[1])-sqrt(adc[5])))/(adc[1]+adc[5]+adc[6]+adc[0]+1.35*adc[3]);
			}
//			else if(circle_inc>320&&circle_inc<=350)
//			{
//			error=0.5;
//			}
			else
			{
				
				buzzer = 0;		//出右圆环蜂鸣器开
				
				
				circle_R = 2;
				x_inc = 0;
				cir_inc=0;
				circle_inc = 0;
				straight_inc = motor_inc;
				
			}
		}
		
		}
	}

	if(circle_R==2)
		{
			
			if(motor_inc-straight_inc<1800)
			{
				error=0;
//				error += -0.13;
				flag_shizi = 0;		//保证圆环内十字不判断
//				return;
			}
			else if(motor_inc-straight_inc>=1800&&motor_inc-straight_inc<2800)
			{
				
			}
			else
			{
				buzzer = 1;		//出右圆环蜂鸣器开
				circle_count+=10;	//右圆环加10
				cricle_count_order++;
				circle_R=0;
				straight_inc=motor_inc;
			}
		}

//------------------------------------------------------------

	 
//////////////避障/////////////////////////////////////避障////////////////////////////////避障////////////////////////////////避障////////////////////////////////

	if(flag_circle==0&&circle_R==0&&circle_L==0&&flag_shizi==0&&flag_shizi_R==0)//&& (error>-0.25 && error<0.25))
	{
		
		
		
		if(Lidar1.distance < 80 && Lidar1.distance > 35&& barrier==0 && fabs(error) < 0.7 && ((bizhang_order >> barrier_count) & 1))
		{
//			if(			bizhang_window[0] < 780
//					&&	bizhang_window[1] < 780
//				)
//			{
			barrier=1;
			//	set_turn = 0.5;
			//	r_target_speed = 10.0;
			//	l_target_speed = 10.0;
			bizhang_inc=motor_inc;
			flag_barrier=1;
			buzzer = 0;
			barrier_count+=1;
			x_inc = 0;
//			}
		}
		else if(Lidar1.distance < 80 && Lidar1.distance > 0&& barrier==0 && fabs(error) < 0.7 && (((bizhang_order >> barrier_count) & 1)==0))
		{
			barrier=2;
			bizhang_inc=motor_inc;
		}
		//-------------------坡道处理-------------------坡道处理-------------------坡道处理-------------------坡道处理-------------------坡道处理
		
		
		
		
		//////////避障处理//////////////////////避障处理////////////////////避障处理////////////////////避障处理////////////////////避障处理//////////
		
		
	}

	if(barrier==2) 
		{
			if(motor_inc-bizhang_inc <= 9000)
			{
			//	r_target_speed = 24.0;
		//		l_target_speed = 24.0;
				buzzer = 0;
			}
			else if(motor_inc-bizhang_inc > 9000)
			{
			//	r_target_speed = 24.0;
		//		l_target_speed = 24.0;
				buzzer = 1;
				barrier = 0;
				barrier_count+=1;
			}
		}
		
		
		
	if(flag_barrier==1)		//左拐
	{
		
		x_inc+=gyro_x*0.005;
		
		if(bizhang==0)		//
		{
			if(x_inc > -34)
			{
				//set_turn = 0.8;
				buzzer = 0;
				error=-3.3;
			}
			else 
			{
				//set_turn = 0.26;
				bizhang_inc = motor_inc;
				bizhang=1;
			}
		}
		
		if(bizhang==1)		//直走
		{
			if(motor_inc-bizhang_inc <= 800)
			{
	//			set_turn = 0.34;
			//	r_target_speed = 31.0;
			//	l_target_speed = 31.0;
				buzzer = 1;
				error=0;
			}
			else if (motor_inc-bizhang_inc > 800)
			{
				
				bizhang=2;
			}
		}
		
//		if(bizhang==2)		//1é?D
//		{
//			if(x_inc < 0)
//			{
//				//set_turn = 0.3;
//				buzzer = 0;
//				error=2.3;
//			}
//			else 
//			{
//				//set_turn = 0.26;
//				bizhang_inc = motor_inc;
//				bizhang=3;
//			}
//		}
//		
//		if(bizhang==3)		 //?±×?
//		{
//			if(motor_inc-bizhang_inc <= 800)
//			{
//				buzzer = 1;
//				error=0;
//			}
//			else if (motor_inc-bizhang_inc > 800)
//			{
//				
//				bizhang=4;
//			}
//		}
		
		if(bizhang==2)		//回正
		{
			if(x_inc < 0)
			{
			//	set_turn = 0.3;
				buzzer = 0;
				error=2.0;
			}
			else 
			{
		//		set_turn = 0.26;
				bizhang_inc = motor_inc;
				bizhang=3;
			}
		}
		
		if(bizhang==3)		//走弧线
		{
			if(motor_inc-bizhang_inc <= 500)
			{
				buzzer = 1;
				error = 0.65;
			
			}
			else if (motor_inc-bizhang_inc > 500)
			{
				bizhang_inc = motor_inc;
				bizhang=4;
			}
		}
		
		
		
		if(bizhang==4)		//关元素循
		{
			if(motor_inc-bizhang_inc <= 800)
			{
				buzzer = 0;
				
			}
			else if (motor_inc-bizhang_inc > 800)
			{
				bizhang=5;
			}
		}
		
		
		if(bizhang==5)	
		{
			
			barrier=0;
			x_inc = 0;
			buzzer = 1;
			bizhang=0;
			flag_barrier=0;
			bizhang_inc = 0;
		}
	}

}











float pow2(float w)
{
	float d = w*w;
	
	return d;
}


//float save_s_r[7] = {0,0,0,0,0,0,0};   //暂时
//int x = 0;





float mask[2][7] = //这些参数怎么拟定的？？？,,手推车
		{	

		//	0.21,0.065,0.025,0.94,0.244,0.119,0.63,		//第一版//一行左圆环，二行右圆环
			//0.526,0.082,0.115,0.95,0.085,0.087,0.309};	
			
		//	0.25,0.06,0.028,0.92,0.26,0.11,0.68,		//第二版
		//	0.42,0.056,0.128,0.95,0.078,0.085,0.315};

		// 0.285,0.06,0.042,1,0.3,0.173,0.65,		//第三版
		// 	0.55,0.11,0.14,0.99,0.068,0.08,0.33};
			
			// 0.3,0.07,0.04,0.996,0.274,0.122,0.444,		//第三版
		 	//0.49,0.043,0.1,0.99,0.084,0.1,0.34};
			
			// 0.23,0.027,0.03,0.86,0.265,0.384,0.9,		//第四版
		// 	0.82,0.043,0.1,0.87,0.078,0.055,0.28};
		
//			 0.39,0.10,0.03,1.023,0.318,0.194,0.68,		//第五版
//		 	0.75,0.093,0.10,1.0,0.12,0.075,0.30};
			
		//	 0.32,0.117,0.093,0.89,0.04,0.075,0.53,		//第六版
		// 	0.642,0.21,0.155,0.854,0.174,0.096,0.286};
		
//			0.28,0.245,0.163,0.87,0.135,0.187,0.445,		//第七版
//		 	0.51,0.255,0.173,0.8,0.145,0.285,0.295};
		
//			0.342,0.08,0.09,0.9,0.022,0.079,0.5,		//第八版
//		 	0.51,0.255,0.01,0.925,0.08,0.242,0.389};
//		
//			0.364,0.25,0.085,1.023,0.073,0.107,0.64,		//第九版室内
//	 	  0.6,0.082,0.027,0.930,0.15,0.214,0.327};
		
//		0.46,0.23,0.097,1.0,0.025,0.1,1.0, 				//第十版横电感室内
//		1.0,0.11,0.087,0.97,0.115,0.235,0.5};
		
//		0.478,0.20,0.035,1.023,0.134,0.178,0.634, 				//第十一版横电感室内
//		0.690,0.088,0.15,1.023,0.162,0.275,0.500};
		
			0.345,0.17,0.365,1.023,0.86,0.144,0.805, 				//第十二外八版电感室内
			0.78,0.088,0.666,1.01,0.405,0.26,0.41};	

char YUANHUAN(void)//用这个以及电感值去判断处理
{
	
			

	
		
	s_r = (pow2(mask[0][0] - adc[0]) + 
				 pow2(mask[0][1] - adc[1]) + 
				 pow2(mask[0][2] - adc[2]) + 
				 pow2(mask[0][3] - adc[3]) + 
				 pow2(mask[0][4] - adc[4]) + 
				 pow2(mask[0][5] - adc[5]) +
				 pow2(mask[0][6] - adc[6])
	);

//	s_r = (
//				 pow2(mask[0][1] - adc[1]) + 
//				 pow2(mask[0][2] - adc[2]) + 
//				 pow2(mask[0][3] - adc[3]) + 
//				 pow2(mask[0][4] - adc[4]) + 
//				 pow2(mask[0][5] - adc[5])
//				 );

//	for(x = 0; x < 7; x++){
//			save_s_r[x] = (mask[0][x] - adc[x]) * (mask[0][x] - adc[x]) *1000;
//	
//	}

		s_r_RYUAN = (pow2(mask[1][0] - adc[0]) + 
				 pow2(mask[1][1] - adc[1]) + 
				 pow2(mask[1][2] - adc[2]) + 
				 pow2(mask[1][3] - adc[3]) + 
				 pow2(mask[1][4] - adc[4]) + 
				 pow2(mask[1][5] - adc[5]) + 
				 pow2(mask[1][6] - adc[6])
	);
	
	
	
	
	
		if(s_r_RYUAN*1000 < HUANDAO_Ryuzhi )//右入环岛
				return 2;
		if(s_r*1000 < HUANDAO_Lyuzhi )//左入环岛
				return 1;
	

	
	
	
	

	else return 0;
}


