///*********************************************************************************************************************
// * COPYRIGHT NOTICE
// * Copyright (c) 2020,��ɿƼ�
// * All rights reserved.
// * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
// *
// * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
// * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
// *
// * @file       		isr
// * @company	   		�ɶ���ɿƼ����޹�˾
// * @author     		��ɿƼ�(QQ790875685)
// * @version    		�鿴doc��version�ļ� �汾˵��
// * @Software 			MDK FOR C251 V5.60
// * @Target core		STC32F12K
// * @Taobao   			https://seekfree.taobao.com/
// * @date       		2020-4-14
// ********************************************************************************************************************/
#include "headfile.h"
#include "../inc/CCD_Driver.h"




//UART1�ж�
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
        //�����Զ�����
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

//UART2�ж�
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
		//�������ݼĴ���Ϊ��S2BUF
		if(wireless_type == WIRELESS_SI24R1)
        {
            wireless_uart_callback();           //����ת���ڻص�����
        }
        else if(wireless_type == WIRELESS_CH9141)
        {
            bluetooth_ch9141_uart_callback();   //����ת���ڻص�����
        }
	}
}


//UART3�ж�
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
		//�������ݼĴ���Ϊ��S3BUF

	}
}


//UART4�ж�
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
		//�������ݼĴ���Ϊ��S4BUF;


	}
}

#define LED P52
void INT0_Isr() interrupt 0
{
	LED = 0;	//����LED
}
void INT1_Isr() interrupt 2
{

}
void INT2_Isr() interrupt 10
{
	INT2_CLEAR_FLAG;  //����жϱ�־
}
void INT3_Isr() interrupt 11
{
	INT3_CLEAR_FLAG;  //����жϱ�־
}

void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;  //����жϱ�־
}

void TM0_Isr() interrupt 1
{

}
void TM1_Isr() interrupt 3
{

}
void TM2_Isr() interrupt 12
{
	TIM2_CLEAR_FLAG;  //����жϱ�־
	
}
void TM3_Isr() interrupt 19
{
	static uint8 TM3_Isr_count  =0;
	static uint8 last_aim_angle_filter = 0;
	TIM3_CLEAR_FLAG; //����жϱ�־
	
	if(car_ready_flag){
		
			//��ȡ��������ֵ
			if(DIR == 1){
			dat = ctimer_count_read(CTIM0_P34)*5;
			}
			else{
				dat = ctimer_count_read(CTIM0_P34)*5* -1;
			}
			//����BEGIN״̬�ٶȣ�����·���жϴ�С������Բ������״̬������circle_begin_distence
			if(count_crbegin_flag == count_start)
				circle_begin_distence += dat;
			else if(count_crbegin_flag == count_clear){
				circle_begin_distence = 0;
			}
			//�µ�
			if(count_ramp_flag == count_start)
				ramp_distence += dat/10;
			else if(count_ramp_flag == count_clear){
				ramp_distence = 0;
			}
			//��������뻷
			if (count_nocri_flag == count_start){//��������ʼ����·��
				circle_no_distence += dat;
				if(circle_no_distence >= 40000){//����һ��·��ֹͣ�ۼ�
					count_nocri_flag = count_stop;
				}
			}
			//ʮ��
			if(count_cross_flag == count_start){
				cross_distence += dat;
				if(cross_distence >= 27000){//����һ��·��ֹͣ�ۼ�
					count_cross_flag = count_stop;
				}
			}
			//ʮ��Բ������·��
			if(count_cross_cricle_flag == count_start){
				cross_cricle_distence += (dat/10);
				//cross_cricle_proccess�б���С����ֵȥ�ж��Ƿ�Բ��ʮ�֣��÷�ΧӦ�Ⱥ�����ֵ��΢��
				if(cross_cricle_distence >= 61000 && flag_cross_circle == CROSS_CIRCLE_NONE){
					count_cross_cricle_flag = count_stop;
				}
			}
			//����
			if(count_turn_pro_flag == count_start){
				tuen_pro_distence += dat;
				if(tuen_pro_distence >= 27000){//����һ��·��ֹͣ�ۼ�
					count_turn_pro_flag = count_stop;
				}
			}
			//���ΰ�����
			if(count_zebra_flag == count_start){
				zebra_distence += dat;
				if(zebra_distence >= 60000){//����һ��·��ֹͣ�ۼ�
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
			//���pid����
			motor_pwm_out = PosPID_realize(&motor_pid,dat);
			
		
		//���ݱ�־λͣ��
		if((P44 == 1 && flag_ramp == RAMP_NONE && zebra_distence > 50000)|| dl1b_distance_mm < 200 ){
			process_flag = 0;
		}
		//���߱���
		if(!process_flag){
			// motor_pwm_out = 0;
			motor_pid.target_val = (float)0;
			aim_angle_filter = 0;
			// P32 = 0;
		}
		if(car_ready_gogogo){//������
			if(dat > 50){
				pwm_duty(PWMB_CH1_P00, 800);//���
				pwm_duty(PWMB_CH2_P01, 800);//���
			}else {
				pwm_duty(PWMB_CH1_P00, 0);//���
				pwm_duty(PWMB_CH2_P01, 0);//���
			}
			//����޷�
			// if(dat < 100)limit_motor_pwm = 1000;
			// else limit_motor_pwm = 4000;
			limit_motor_pwm = 4000;
			if(motor_pwm_out >= limit_motor_pwm)motor_pwm_out = limit_motor_pwm;
			if(motor_pwm_out <= -limit_motor_pwm)motor_pwm_out = -limit_motor_pwm;
			if(motor_pwm_out <= limit_motor_pwm&&motor_pwm_out >= 0){//��ת
				pwm_duty(PWMA_CH3P_P24, motor_pwm_out);//���
				P26 = 0;
			}
			if(motor_pwm_out < 0&&motor_pwm_out >= -limit_motor_pwm){//��ת
				pwm_duty(PWMA_CH3P_P24, -motor_pwm_out);//���
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
//!!!!!!!!!!!!!!!!!!���!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	if(steer_pwm_out >= 798)steer_pwm_out = 798;
	if(steer_pwm_out <= 453)steer_pwm_out = 453;
	pwm_duty(PWMB_CH3_P33, steer_pwm_out);//���
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	}else {
		pwm_duty(PWMB_CH3_P33, steet_midlle);//���
	}
	
}
extern void pit_callback(void);
void TM4_Isr() interrupt 20
{
	TIM4_CLEAR_FLAG; //����жϱ�־	
	
	//��ȡtof��ֵ

	dl1b_get_distance();
	if(dl1b_finsh_flag){
		dl1b_finsh_flag = 0;
	}



///////////////������//////////////////
	imu660ra_get_gyro();
    //ȥ��Ư
    // imu660ra_gyro_x -= 1.2;
	X_gyro = imu660ra_gyro_transition(imu660ra_gyro_x);
	Y_gyro = imu660ra_gyro_transition(imu660ra_gyro_y);
	Z_gyro = imu660ra_gyro_transition(imu660ra_gyro_z);

//��ͨ�˲�
    // if(X_gyro<2&&X_gyro>-2){
    //     X_gyro=0;
    // }

    yaw += Z_gyro *0.005;                               //����ƫ����
    if(yaw<0)yaw+=360;
    else if(yaw>360)yaw=0;

//���Ե�����ֱ�ߣ�
	get_inertia_offset();

///////////////ccd//////////////////
	//CCD�ɼ�����
	ccd_collect();	 
	
	ccd_value1 = ccd_threshold(ccd_data_ch1);
	ccd_value2 = ccd_threshold(ccd_data_ch2);
	
	if((flag_cross != CROSS_NONE) || (flag_circle != CIRCLE_NONE))ccd_value2 = element_ccd_value;
	else {element_ccd_value = ccd_value2;}
	
	ccd_two_value(ccd_value2, ccd_value2);
	Bin_CCD_Filter();
	//�����߿�ȼ���
	count_ones_ccd1 = count_ones(ccd_data_01,128);
	count_ones_ccd2 = count_ones(ccd_data_02,128);


	if(car_ready_flag){//��������Ԫ�ؼ��
		//�µ�
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

	//ʮ���ж�
		if(((count_ones_ccd1>89 && count_ones_ccd2 < 70) || 
			(count_ones_ccd2 > 105 && count_ones_ccd1 < 50)) && 
			flag_ramp == RAMP_NONE && 
		flag_circle != CIRCLE_LEFT_IN && flag_circle != CIRCLE_LEFT_OUT&&
		flag_circle != CIRCLE_LEFT_RUNNING && flag_circle != CIRCLE_LEFT_END && 
		flag_circle != CIRCLE_RIGHT_IN && flag_circle != CIRCLE_RIGHT_OUT&&
		flag_circle != CIRCLE_RIGHT_RUNNING && flag_circle != CIRCLE_RIGHT_END &&
		flag_cross_circle == CROSS_CIRCLE_NONE && cross_check)
		{
			count_cross_cricle_flag = count_start;//һ��·�����ж�ʮ��Բ��
			cross_cricle_distence = 0;
			count_cross_flag = count_start;
			cross_distence = 0;
			cross_circle_yaw = yaw;//��¼��ʮ��Բ���ĽǶ�
			flag_cross = CROSS_RUNNING;
		}
	//ʮ��Բ��
		if(cross_circle_cleck){
			cross_cricle_proccess();
		}
	//s�����⴦�����������ռ���
		if(flag_ramp == RAMP_NONE && flag_cross_circle == CROSS_CIRCLE_NONE && flag_cross == CROSS_NONE && flag_circle == CIRCLE_NONE )//�����κ�һ��Ԫ��
		{
			//��������ͬһ�������ƫ��ֵ����¼��ʼ�Ƕȣ�ƫ��ֵ��ͬ������ˢ�½Ƕ�
			if(last_offset > 10 && offset2 > 10){//�����Ҹ�
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



///////////////����//////////////////
	get_mid_point1();//ccd1 mid
	get_mid_point2();//ccd2 mid
	
	last_left_point1 = left_point1;
	last_right_point1 = right_point1;

	get_offset();//�������  offset+������  -���Ҵ�� 
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