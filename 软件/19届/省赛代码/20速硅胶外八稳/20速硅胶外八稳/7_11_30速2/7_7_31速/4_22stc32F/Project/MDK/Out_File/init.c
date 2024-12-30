#include "headfile.h"

//adc获取值
float adc[11]={0} ;

//陀螺仪变量
int init_gyro_x = 0,init_gyro_y = 0,init_gyro_z = 0;
float new_gyro_x = 0,new_gyro_y = 0,new_gyro_z = 0;
float z_inc = 0,x_inc=0,y_inc=0;
float angle_x;

float last_acc_x,last_acc_y,last_acc_z;
float last_gyro_x,last_gyro_y,last_gyro_z;

float Xdata=0.0,Ydata=0.0,Zdata=0.0;
float acc_x=0.0,acc_y=0.0,acc_z=0.0;
float gyro_x=0.0,gyro_y=0.0,gyro_z=0.0;

int text=0;//text零漂标志位
float alpha = 0.3;

char a=0;

//电机pwm输出
float r_pwm=0.0,l_pwm=0.0;
float last_r_pwm=0.0,last_l_pwm=0.0;


void car_pre_init(void)
{
	
	//初始化PID
	PID_init(&mr_pid); 			//角速度环
	PID_init(&ml_pid);
	PID_init(&R_pid);  			//速度环
	PID_init(&L_pid);
	PID_init(&turn_pid);				//角速度环
//	PID_init(&Steer_pid);

	PID_Set(&R_pid,200,25,17);				
	PID_Set(&L_pid,200,25,17);			//11速速度环pid60,18,20 			14速度140,57  ，	//速度环
//	PID_Set(&mr_pid,1,1,0);				
//	PID_Set(&ml_pid,1,1,0);			//  //角度环
	
	PID_Set(&w_pid,0.7,0.1,0);				//角速度环	20 10
//	PID_Set(&turn_pid,162,19,0);				//误差转向环		120，14
	PID_turnSet(&turn_pid,140,7,12,8);				//误差转向环   260,27,0

	
	//初始化ADC
	adc_init(ADC_P01, ADC_SYSclk_DIV_2);	//初始化ADC,P0.6通道 ，ADC时钟频率：SYSclk/2
	adc_init(ADC_P00, ADC_SYSclk_DIV_2);	//初始化ADC,P1.1通道 ，ADC时钟频率：SYSclk/2
	adc_init(ADC_P02, ADC_SYSclk_DIV_2);	//初始化ADC,P1.4通道 ，ADC时钟频率：SYSclk/2
	adc_init(ADC_P15, ADC_SYSclk_DIV_2);	//初始化ADC,P1.5通道 ，ADC时钟频率：SYSclk/2
	adc_init(ADC_P14, ADC_SYSclk_DIV_2);	//初始化ADC,P0.6通道 ，ADC时钟频率：SYSclk/2
	adc_init(ADC_P11, ADC_SYSclk_DIV_2);	//初始化ADC,P1.1通道 ，ADC时钟频率：SYSclk/2
	adc_init(ADC_P05, ADC_SYSclk_DIV_2);	//初始化ADC,P1.4通道 ，ADC时钟频率：SYSclk/2
	adc_init(ADC_P06, ADC_SYSclk_DIV_2);	//初始化ADC,P1.5通道 ，ADC时钟频率：SYSclk/2
	delay_ms(10);
	
		//电机PWM输出
//	pwm_init(MOTOR1_CH1,15000,0);		//左电机   7971
//	pwm_init(MOTOR1_CH2,15000,0);		//左电机	
//	pwm_init(MOTOR2_CH1,15000,0);		//左电机   7971
//	pwm_init(MOTOR2_CH2,15000,0);		//左电机	
//	delay_ms(10);

	pwm_init(MOTOR1, 15000, 0);
	pwm_init(MOTOR2, 15000, 0);
	delay_ms(10);
	
//	//初始化串口
//	uart_init(UART_2,UART_RX,UART_TX,BOTRATE,UART_TIM);//TIM_2,波特率为9600
	
	//初始化IPS屏幕
		ips114_init();	
		
	//编码器初始化
	ctimer_count_init(MOTOR1_ENCODER);
	ctimer_count_init(MOTOR2_ENCODER);
	delay_ms(30);
	
//测距模块初始化	
//	while(dl1b_init());
	//陀螺仪初始化
	//	TF Lidar1;
	Lidar1.Address = 0x10;
	User_I2C_Init();
	imu660ra_init();
	
	target_speed_save = l_target_speed;
	return;
}


void get_adc(void)	//ADC获取并处理
{
	int adc_get_value[7] = {0};	
	int i = 0 ,k = 0;
	for( i = 0;i < 5;++i)
	{
		adc_get_value[0] += adc_once(ADC0,ADC_10BIT);
		adc_get_value[1] += adc_once(ADC1,ADC_10BIT);
		adc_get_value[2] += adc_once(ADC2,ADC_10BIT);
		adc_get_value[3] += adc_once(ADC3,ADC_10BIT);
		adc_get_value[4] += adc_once(ADC4,ADC_10BIT);//5
		adc_get_value[5] += adc_once(ADC5,ADC_10BIT);//6
		adc_get_value[6] += adc_once(ADC6,ADC_10BIT);//7	
	}
	for( k = 0;k < 7;++k)
	 {
		 adc[k] = (float)adc_get_value[k] / (5*1000.0f);
	 }
}


//--------------------------------归一化    

////float save_max_adc[7] = {0,0,0,0,0,0,0};
////float save_min_adc[7] = {10000.0,10000.0,10000.0,10000.0,10000.0,10000.0,10000.0}; 


//float save_max_adc[7] = {785, 568, 714, 582, 710, 569, 662};
//float save_min_adc[7] = {8, 7, 23, 2, 9, 7, 25}; 
////float save_min_adc[7] = {0};
//int target_adc = 0;

//void get_adc(void)	//ADC获取并处理
//{
//	int adc_get_value[7] = {0};	
//	int i = 0 ,k = 0;
//	for( i = 0;i < 5;++i)
//	{
//		adc_get_value[0] += adc_once(ADC0,ADC_10BIT);
//		adc_get_value[1] += adc_once(ADC1,ADC_10BIT);
//		adc_get_value[2] += adc_once(ADC2,ADC_10BIT);
//		adc_get_value[3] += adc_once(ADC3,ADC_10BIT);
//		adc_get_value[4] += adc_once(ADC4,ADC_10BIT);//5
//		adc_get_value[5] += adc_once(ADC5,ADC_10BIT);//6
//		adc_get_value[6] += adc_once(ADC6,ADC_10BIT);//7	
//		
//	}
//	
//	switch (target_adc){
//			case 0:
//					
//					break;
//			case 1:
//					if (save_max_adc[0] < adc_once(ADC0,ADC_10BIT)){
//							save_max_adc[0] = adc_once(ADC0,ADC_10BIT);
//					}
//					if(save_min_adc[0] > adc_once(ADC0,ADC_10BIT)){
//							save_min_adc[0] = adc_once(ADC0,ADC_10BIT);
//					}
//					break;
//			case 2:
//					if (save_max_adc[1] < adc_once(ADC1,ADC_10BIT)){
//							save_max_adc[1] = adc_once(ADC1,ADC_10BIT);
//					}
//					if(save_min_adc[1] > adc_once(ADC1,ADC_10BIT)){
//							save_min_adc[1] = adc_once(ADC1,ADC_10BIT);
//					}
//					break;
//			case 3:
//					if (save_max_adc[2] < adc_once(ADC2,ADC_10BIT)){
//							save_max_adc[2] = adc_once(ADC2,ADC_10BIT);
//					}
//					if(save_min_adc[2] > adc_once(ADC2,ADC_10BIT)){
//							save_min_adc[2] = adc_once(ADC2,ADC_10BIT);
//					}
//					break;
//			case 4:
//					if (save_max_adc[3] < adc_once(ADC3,ADC_10BIT)){
//							save_max_adc[3] = adc_once(ADC3,ADC_10BIT);
//					}
//					if(save_min_adc[3] > adc_once(ADC3,ADC_10BIT)){
//							save_min_adc[3] = adc_once(ADC3,ADC_10BIT);
//					}
//					break;
//			case 5:
//					if (save_max_adc[4] < adc_once(ADC4,ADC_10BIT)){
//							save_max_adc[4] = adc_once(ADC4,ADC_10BIT);
//					}
//					if(save_min_adc[4] > adc_once(ADC4,ADC_10BIT)){
//							save_min_adc[4] = adc_once(ADC4,ADC_10BIT);
//					}
//					break;
//			case 6:
//					if (save_max_adc[5] < adc_once(ADC5,ADC_10BIT)){
//							save_max_adc[5] = adc_once(ADC5,ADC_10BIT);
//					}
//					if(save_min_adc[5] > adc_once(ADC5,ADC_10BIT)){
//							save_min_adc[5] = adc_once(ADC5,ADC_10BIT);
//					}
//					break;
//			case 7:
//					if (save_max_adc[6] < adc_once(ADC6,ADC_10BIT)){
//							save_max_adc[6] = adc_once(ADC6,ADC_10BIT);
//					}
//					if(save_min_adc[6] > adc_once(ADC6,ADC_10BIT)){
//							save_min_adc[6] = adc_once(ADC6,ADC_10BIT);
//					}
//					break;
//			}			
//	
//		
//		for (k = 0; k < 7; k++){
//				adc[k] = (float)( ((adc_get_value[k] / 5) - save_min_adc[k])  /
//													(save_max_adc[k] - save_min_adc[k]) 
//												);
//		}
//}



//-------------------------------






void gyroOffsetInit(void)     //去零漂，
{
	  if(text==0)
		{
				//连续读取100次陀螺仪的X轴数据
				if (a<100) 
			 {
					  imu660ra_get_acc();
            imu660ra_get_gyro();
						Xdata+=imu660ra_gyro_x;
//						Ydata+=imu660ra_gyro_y;
//						Zdata+=imu660ra_gyro_z;
						delay_ms(5);    // 最大 1Khz
				    a++;
//				    ips114_showint16(0,0,a);
//				 		ips114_showint16(0,7,ips_page);
				}
				else if(a ==100)
				{
					Xdata *= 0.01;	//当读取完100次后，Xdata 被除以100（即乘以0.01），以得到平均偏移量
//					Ydata *= 0.01;
//					Zdata *= 0.01;
//					ips114_clear(WHITE);									//清屏
					ips_page=1;
					a++;					
					text=1;
//					oled_p8x16str(80,7,"   ");
//					oled_int16(3,4,text);
				}

	}
		else if(text==1)
		{
//			imu660ra_get_gyro();
		imu660ra_get_acc(); 		//获取加速度数据
		imu660ra_get_gyro();    	//获取陀螺仪数据
			gyro_x = imu660ra_gyro_transition(imu660ra_gyro_x - Xdata)* alpha+last_gyro_x*(1 - alpha); 
			last_gyro_x=gyro_x;

//		  duoji();
//			motor();//电机使用
			
//			imu660ra_get_acc();
      
//			acc_x = (imu660ra_acc_transition(imu660ra_acc_x) * alpha)  + last_acc_x * (1 - alpha);
//			acc_y = (imu660ra_acc_transition(imu660ra_acc_y) * alpha)  + last_acc_y * (1 - alpha);
//			acc_z = (imu660ra_acc_transition(imu660ra_acc_z) * alpha)  + last_acc_z * (1 - alpha);
////			acc_x=(float)imu660ra_acc_x*0.7+last_acc_x*0.3;
////			acc_y=(float)imu660ra_acc_y*0.7+last_acc_y*0.3;
////			acc_z=(float)imu660ra_acc_z*0.7+last_acc_z*0.3;
//			last_acc_x=acc_x;
//			last_acc_y=acc_y;
//			last_acc_z=acc_z;
//			

//			//! 陀螺仪角速度必须转换为弧度制角速度: deg/s -> rad/s

//			gyro_x = (imu660ra_gyro_transition(imu660ra_gyro_x )-imu660ra_gyro_transition(Xdata ) )* alpha+last_gyro_x* (1 - alpha);
//			gyro_y = (imu660ra_gyro_transition(imu660ra_gyro_y )-imu660ra_gyro_transition(Ydata ) )* alpha+last_gyro_y* (1 - alpha);
			
//			gyro_z = ((float)(imu660ra_gyro_z-Zdata) /65.6 )* (float)alpha+last_gyro_z* (1 - alpha);
//			
//			gyro_z = imu660ra_gyro_transition(imu660ra_gyro_z-Zdata ) * alpha+last_gyro_z* (1 - alpha);

//			last_gyro_z=gyro_z;
//			last_gyro_y=gyro_y;

			
		}
		
}

