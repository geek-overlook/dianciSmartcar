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
#define LED P52

//���
float error=0,error_heng=0,error_xie=0,error_shu=0,pow_error=0,new_error=0,error_shu2=0;



//���߱���
char distance_protect=0;		//1������0ֹͣ
extern char flag_distance_protect;  //�������߱���
	
	
extern char flag_zhijiao;
int judge_sign = 1;	//Ԫ���ж�	1������0��ֹ	

float error_shuA;
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




volatile uint32 SysTickFlag = 0; // ��������ʼÿ1ms��1			//volatile ȷ������ָ��ᱻ���������Ż�������
unsigned char temp_count = 0;
void TM0_Isr() interrupt 1
{
	
}

void TM1_Isr() interrupt 3
{
		gyroOffsetInit();  
		LunaReadDist(&Lidar1);

		
		
//		else if(dl1b_distance_mm>=450)
//		{
//			barrier=0;
//		}
	
		//���ֵ����
		error = (5.3*(sqrt(adc[0])-sqrt(adc[6]))+5.2*(sqrt(adc[1])-sqrt(adc[5])))/(adc[1]+adc[5]+adc[6]+adc[0]+5.4*adc[3]);//2.6   3.27   2.8
	
		//error = (+2.3*(sqrt(adc[1])-sqrt(adc[5])))/(adc[1]+adc[5]+2.1*adc[3]);		//����б���
	
		error_xie=(sqrt(adc[0])-sqrt(adc[6]))/(adc[6]+adc[0]+adc[3]);
	  error_shu=(sqrt(adc[1])-sqrt(adc[5]))/(adc[1]+adc[5]+adc[3]);
    error_shu2=(sqrt(adc[2])-sqrt(adc[4]))/(adc[2]+adc[4]+adc[3]);
//		error_shuA = (sqrt(((adc[1]-0.5)*(adc[1]-0.5) + (adc[2]-0.5)*(adc[2]-0.5) + (adc[4]-0.5)*(adc[4]-0.5) + (adc[5]-0.5)*(adc[5]-0.5))/4));
	
	
//		if(error>=0)
//		{
//			new_error=1.4*error*error;
//		}
//		else
//		{
//			new_error=-1.4*error*error;
//		}
		  gyro_x=gyro_x/10*10;
//			x_inc+=gyro_x*0.01;
		get_adc();

		if (judge_sign == 1)		//�����水������
		{		judge();	}
//		
	if(text==1 && motor_start==1)			//��Ư����
	{
	  motor();

//		multistage_pid();
	}
	
	
	if(((adc[0]+adc[1]+adc[2]+adc[3]+adc[4]+adc[5]+adc[6])*1000>35)||(flag_barrier==1))
	{
		distance_protect=1;
	}
	else
	{
		distance_protect=0;
	}
	

//		imu660ra_get_acc(); 		//��ȡ���ٶ�����
//		imu660ra_get_gyro();    	//��ȡ����������

//		x_inc+=gyro_x*0.01;
//		y_inc+=gyro_y*0.01;
//		z_inc+=gyro_z*0.01;
	
//	if(temp_count++ > 100)
//	{
//		LED = !LED;
//		temp_count = 0;
//	}
	SysTickFlag++;
	P04 = !P04;
}
void TM2_Isr() interrupt 12
{
	TIM2_CLEAR_FLAG;  //����жϱ�־
	
}
void TM3_Isr() interrupt 19
{
	TIM3_CLEAR_FLAG; //����жϱ�־
	
}
extern void pit_callback(void);
void TM4_Isr() interrupt 20
{
	TIM4_CLEAR_FLAG; //����жϱ�־
//	ccd_collect();	 //CCD�ɼ�����
//	pit_callback();
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