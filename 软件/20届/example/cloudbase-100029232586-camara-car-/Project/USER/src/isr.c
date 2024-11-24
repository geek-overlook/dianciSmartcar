///*********************************************************************************************************************
// * COPYRIGHT NOTICE
// * Copyright (c) 2020,逐飞科技
// * All rights reserved.
// * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
// *
// * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
// * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
// *
// * @file       		isr
// * @company	   		成都逐飞科技有限公司
// * @author     		逐飞科技(QQ790875685)
// * @version    		查看doc内version文件 版本说明
// * @Software 			MDK FOR C251 V5.60
// * @Target core		STC32F12K
// * @Taobao   			https://seekfree.taobao.com/
// * @date       		2020-4-14
// ********************************************************************************************************************/
#include "headfile.h"
#include "../inc/CCD_Driver.h"




//UART1中断
void UART1_Isr() interrupt 4
{
    uint8 res;
	static uint8 dwon_count;
    if(UART1_GET_TX_FLAG)
    {
        UART1_CLEAR_TX_FLAG;
        busy[1] = 0;
    }
    if(UART1_GET_RX_FLAG)
    {
        UART1_CLEAR_RX_FLAG;
        res = SBUF;
        //程序自动下载
        if(res == 0x7F)
        {
            if(dwon_count++ > 20)
                IAP_CONTR = 0x60;
        }
        else
        {
            dwon_count = 0;
        }
    }
}

//UART2中断
void UART2_Isr() interrupt 8
{
    if(UART2_GET_TX_FLAG)
	{
        UART2_CLEAR_TX_FLAG;
		busy[2] = 0;
	}
    if(UART2_GET_RX_FLAG)
	{
        UART2_CLEAR_RX_FLAG;
		//接收数据寄存器为：S2BUF
		if(wireless_type == WIRELESS_SI24R1)
        {
            wireless_uart_callback();           //无线转串口回调函数
        }
        else if(wireless_type == WIRELESS_CH9141)
        {
            bluetooth_ch9141_uart_callback();   //蓝牙转串口回调函数
        }
	}
}


//UART3中断
void UART3_Isr() interrupt 17
{
    if(UART3_GET_TX_FLAG)
	{
        UART3_CLEAR_TX_FLAG;
		busy[3] = 0;
	}
    if(UART3_GET_RX_FLAG)
	{
        UART3_CLEAR_RX_FLAG;
		//接收数据寄存器为：S3BUF

	}
}


//UART4中断
void UART4_Isr() interrupt 18
{
    if(UART4_GET_TX_FLAG)
	{
        UART4_CLEAR_TX_FLAG;
		busy[4] = 0;
	}
    if(UART4_GET_RX_FLAG)
	{
        UART4_CLEAR_RX_FLAG;
		//接收数据寄存器为：S4BUF;


	}
}

#define LED P52
void INT0_Isr() interrupt 0
{
	LED = 0;	//点亮LED
}
void INT1_Isr() interrupt 2
{

}
void INT2_Isr() interrupt 10
{
	INT2_CLEAR_FLAG;  //清除中断标志
}
void INT3_Isr() interrupt 11
{
	INT3_CLEAR_FLAG;  //清除中断标志
}

void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;  //清除中断标志
}

void TM0_Isr() interrupt 1
{

}
void TM1_Isr() interrupt 3
{

}
void TM2_Isr() interrupt 12
{
	TIM2_CLEAR_FLAG;  //清除中断标志
	
}
void TM3_Isr() interrupt 19
{
	static uint8 TM3_Isr_count  =0;
	static uint8 last_aim_angle_filter = 0;
	TIM3_CLEAR_FLAG; //清除中断标志
	
	if(car_ready_flag){
		
			//读取编码器数值
			if(DIR == 1){
			dat = ctimer_count_read(CTIM0_P34)*5;
			}
			else{
				dat = ctimer_count_read(CTIM0_P34)*5* -1;
			}
			//积分BEGIN状态速度，根据路程判断大小环，在圆环结束状态中置零circle_begin_distence
			if(count_crbegin_flag == count_start)
				circle_begin_distence += dat;
			else if(count_crbegin_flag == count_clear){
				circle_begin_distence = 0;
			}
			//坡道
			if(count_ramp_flag == count_start)
				ramp_distence += dat/10;
			else if(count_ramp_flag == count_clear){
				ramp_distence = 0;
			}
			//避免二次入环
			if (count_nocri_flag == count_start){//出环岛后开始计算路程
				circle_no_distence += dat;
				if(circle_no_distence >= 40000){//大于一定路程停止累计
					count_nocri_flag = count_stop;
				}
			}
			//十字
			if(count_cross_flag == count_start){
				cross_distence += dat;
				if(cross_distence >= 27000){//大于一定路程停止累计
					count_cross_flag = count_stop;
				}
			}
			//十字圆环检测的路程
			if(count_cross_cricle_flag == count_start){
				cross_cricle_distence += (dat/10);
				//cross_cricle_proccess中变量小于阈值去判断是否圆环十字，该范围应比函数阈值稍微大
				if(cross_cricle_distence >= 61000 && flag_cross_circle == CROSS_CIRCLE_NONE){
					count_cross_cricle_flag = count_stop;
				}
			}
			//急弯
			if(count_turn_pro_flag == count_start){
				tuen_pro_distence += dat;
				if(tuen_pro_distence >= 27000){//大于一定路程停止累计
					count_turn_pro_flag = count_stop;
				}
			}
			//二次斑马线
			if(count_zebra_flag == count_start){
				zebra_distence += dat;
				if(zebra_distence >= 60000){//大于一定路程停止累计
					count_zebra_flag = count_stop;
				}
			}


			ctimer_count_clean(CTIM0_P34);

			if(dat > 500 && dat <= 700){				
				motor_pid.Kp = 12 ;
				motor_pid.Ki = 3;
			}
			else if(dat > 700 ){
				motor_pid.Kp = 13.5;
				motor_pid.Ki = 3.9;
			}
			else if(dat <=500 ){
				motor_pid.Kp = 6.1;
				motor_pid.Ki = 2.7;
			}
			//电机pid计算
			motor_pwm_out = PosPID_realize(&motor_pid,dat);
			
		
		//根据标志位停车
		if((P44 == 1 && flag_ramp == RAMP_NONE && zebra_distence > 50000)|| dl1b_distance_mm < 200 ){
			process_flag = 0;
		}
		//丢线保护
		if(!process_flag){
			// motor_pwm_out = 0;
			motor_pid.target_val = (float)0;
			aim_angle_filter = 0;
			// P32 = 0;
		}
		if(car_ready_gogogo){//允许发车
			if(dat > 50){
				pwm_duty(PWMB_CH1_P00, 800);//电调
				pwm_duty(PWMB_CH2_P01, 800);//电调
			}else {
				pwm_duty(PWMB_CH1_P00, 0);//电调
				pwm_duty(PWMB_CH2_P01, 0);//电调
			}
			//电机限幅
			// if(dat < 100)limit_motor_pwm = 1000;
			// else limit_motor_pwm = 4000;
			limit_motor_pwm = 4000;
			if(motor_pwm_out >= limit_motor_pwm)motor_pwm_out = limit_motor_pwm;
			if(motor_pwm_out <= -limit_motor_pwm)motor_pwm_out = -limit_motor_pwm;
			if(motor_pwm_out <= limit_motor_pwm&&motor_pwm_out >= 0){//正转
				pwm_duty(PWMA_CH3P_P24, motor_pwm_out);//电机
				P26 = 0;
			}
			if(motor_pwm_out < 0&&motor_pwm_out >= -limit_motor_pwm){//反转
				pwm_duty(PWMA_CH3P_P24, -motor_pwm_out);//电机
				P26 = 1;
			}
		}


		if(flag_ramp != RAMP_NONE && (aim_angle_filter >5 || aim_angle_filter < -5))
			aim_angle_filter = last_aim_angle_filter;

		offset_r_x = X_gyro;
		offset_r_d = dat;
		if(offset_r_x < 0)offset_r_x = -offset_r_x;
		if(offset_r_d < 0)offset_r_d = -offset_r_d;

		offset_r = offset_r_x / offset_r_d * 54.f;
		if(offset_r < 0) offset_r = -offset_r;

		inertia_pid.target_val = 20;
		steer_pid.target_val = aim_angle_filter;
		steer_pwm_out = (int)(PosPID_realize( &steer_pid , 0 ) + steet_midlle);

		// steer_pwm_out = (int)(-pid_realize_a(-offset_inertia, 0.0f, steer_pid.Kp, steer_pid.Kd) + steet_midlle);

		last_aim_angle_filter = aim_angle_filter;

	}

			
			
			
	if(process_flag){
//!!!!!!!!!!!!!!!!!!勿改!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	if(steer_pwm_out >= 798)steer_pwm_out = 798;
	if(steer_pwm_out <= 453)steer_pwm_out = 453;
	pwm_duty(PWMB_CH3_P33, steer_pwm_out);//舵机
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	}else {
		pwm_duty(PWMB_CH3_P33, steet_midlle);//舵机
	}
	
}
extern void pit_callback(void);
void TM4_Isr() interrupt 20
{
	TIM4_CLEAR_FLAG; //清除中断标志	
	
	//读取tof数值

	dl1b_get_distance();
	if(dl1b_finsh_flag){
		dl1b_finsh_flag = 0;
	}



///////////////陀螺仪//////////////////
	imu660ra_get_gyro();
    //去零漂
    // imu660ra_gyro_x -= 1.2;
	X_gyro = imu660ra_gyro_transition(imu660ra_gyro_x);
	Y_gyro = imu660ra_gyro_transition(imu660ra_gyro_y);
	Z_gyro = imu660ra_gyro_transition(imu660ra_gyro_z);

//低通滤波
    // if(X_gyro<2&&X_gyro>-2){
    //     X_gyro=0;
    // }

    yaw += Z_gyro *0.005;                               //积分偏航角
    if(yaw<0)yaw+=360;
    else if(yaw>360)yaw=0;

//惯性导航（直线）
	get_inertia_offset();

///////////////ccd//////////////////
	//CCD采集数据
	ccd_collect();	 
	
	ccd_value1 = ccd_threshold(ccd_data_ch1);
	ccd_value2 = ccd_threshold(ccd_data_ch2);
	
	if((flag_cross != CROSS_NONE) || (flag_circle != CIRCLE_NONE))ccd_value2 = element_ccd_value;
	else {element_ccd_value = ccd_value2;}
	
	ccd_two_value(ccd_value2, ccd_value2);
	Bin_CCD_Filter();
	//最大白线宽度计算
	count_ones_ccd1 = count_ones(ccd_data_01,128);
	count_ones_ccd2 = count_ones(ccd_data_02,128);


	if(car_ready_flag){//允许发车后元素检测
		//坡道
		if(dl1b_distance_mm < 500 && flag_ramp == RAMP_NONE && flag_circle == CIRCLE_NONE && ramp_check){
			count_ramp_flag = count_start;
			flag_ramp = RAMP_IN_UP;
			get_target_yaw(offset2);
			//test0 = 1;
		}
		else if(flag_ramp == RAMP_IN_UP && dl1b_distance_mm > 1000){
			ramp_distence = 0;
			count_ramp_flag = count_start;
			flag_ramp = RAMP_IN_MID;
			//test0 = 2;
		}
		else if(flag_ramp == RAMP_IN_MID && dl1b_distance_mm < 490){
			ramp_distence = 0;
			count_ramp_flag = count_start;
			flag_ramp = RAMP_IN_DOWN;
			//test0 = 3;
		}
		else if(flag_ramp == RAMP_IN_DOWN && dl1b_distance_mm > 570){
			flag_ramp = RAMP_NONE;
			count_ramp_flag = count_clear;
			//test0 = 4;
		}
		else if(ramp_distence > 10000){
			flag_ramp = RAMP_NONE;
			count_ramp_flag = count_clear;
			//test0 = 0;
		}

	//十字判断
		if(((count_ones_ccd1>89 && count_ones_ccd2 < 70) || 
			(count_ones_ccd2 > 105 && count_ones_ccd1 < 50)) && 
			flag_ramp == RAMP_NONE && 
		flag_circle != CIRCLE_LEFT_IN && flag_circle != CIRCLE_LEFT_OUT&&
		flag_circle != CIRCLE_LEFT_RUNNING && flag_circle != CIRCLE_LEFT_END && 
		flag_circle != CIRCLE_RIGHT_IN && flag_circle != CIRCLE_RIGHT_OUT&&
		flag_circle != CIRCLE_RIGHT_RUNNING && flag_circle != CIRCLE_RIGHT_END &&
		flag_cross_circle == CROSS_CIRCLE_NONE && cross_check)
		{
			count_cross_cricle_flag = count_start;//一段路程内判断十字圆环
			cross_cricle_distence = 0;
			count_cross_flag = count_start;
			cross_distence = 0;
			cross_circle_yaw = yaw;//记录进十字圆环的角度
			flag_cross = CROSS_RUNNING;
		}
	//十字圆环
		if(cross_circle_cleck){
			cross_cricle_proccess();
		}
	//s弯特殊处理，抑制连续拐急弯
		if(flag_ramp == RAMP_NONE && flag_cross_circle == CROSS_CIRCLE_NONE && flag_cross == CROSS_NONE && flag_circle == CIRCLE_NONE )//不在任何一个元素
		{
			//连续处于同一个方向的偏差值，记录初始角度，偏差值不同方向则刷新角度
			if(last_offset > 10 && offset2 > 10){//左正右负
				turn_pro_flag = 1;
			}
			else if(last_offset < -10 && offset2 < -10){
				turn_pro_flag = 0;
			}
			else {
				turn_pro_yaw = yaw;
			}

			if(yaw_error(turn_pro_yaw,turn_pro_flag) > 170 && flag_turn_pro ==0){
				cross_distence = 0;
				count_turn_pro_flag = count_start;
				flag_turn_pro = 1;
			}
			if(cross_distence > 25000){
				flag_turn_pro = 0;
			}
		}else{
			flag_turn_pro = 0;
		}
	}



///////////////控制//////////////////
	get_mid_point1();//ccd1 mid
	get_mid_point2();//ccd2 mid
	
	last_left_point1 = left_point1;
	last_right_point1 = right_point1;

	get_offset();//计算误差  offset+向左打角  -向右打角 
	last_count_ones_ccd1 = count_ones_ccd1;
	last_count_ones_ccd2 = count_ones_ccd2;
}

//void  INT0_Isr()  interrupt 0;
//void  TM0_Isr()   interrupt 1;
//void  INT1_Isr()  interrupt 2;
//void  TM1_Isr()   interrupt 3;
//void  UART1_Isr() interrupt 4;
//void  ADC_Isr()   interrupt 5;
//void  LVD_Isr()   interrupt 6;
//void  PCA_Isr()   interrupt 7;
//void  UART2_Isr() interrupt 8;
//void  SPI_Isr()   interrupt 9;
//void  INT2_Isr()  interrupt 10;
//void  INT3_Isr()  interrupt 11;
//void  TM2_Isr()   interrupt 12;
//void  INT4_Isr()  interrupt 16;
//void  UART3_Isr() interrupt 17;
//void  UART4_Isr() interrupt 18;
//void  TM3_Isr()   interrupt 19;
//void  TM4_Isr()   interrupt 20;
//void  CMP_Isr()   interrupt 21;
//void  I2C_Isr()   interrupt 24;
//void  USB_Isr()   interrupt 25;
//void  PWM1_Isr()  interrupt 26;
//void  PWM2_Isr()  interrupt 27;