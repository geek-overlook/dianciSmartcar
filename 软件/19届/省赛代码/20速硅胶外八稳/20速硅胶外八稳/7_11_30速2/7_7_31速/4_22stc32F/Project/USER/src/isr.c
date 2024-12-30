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
#define LED P52

//误差
float error=0,error_heng=0,error_xie=0,error_shu=0,pow_error=0,new_error=0,error_shu2=0;



//脱线保护
char distance_protect=0;		//1启动，0停止
extern char flag_distance_protect;  //启动脱线保护
	
	
extern char flag_zhijiao;
int judge_sign = 1;	//元素判断	1开启，0禁止	

float error_shuA;
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




volatile uint32 SysTickFlag = 0; // 从启动开始每1ms加1			//volatile 确保本条指令不会被编译器的优化而忽略
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
	
		//电感值作差
		error = (5.3*(sqrt(adc[0])-sqrt(adc[6]))+5.2*(sqrt(adc[1])-sqrt(adc[5])))/(adc[1]+adc[5]+adc[6]+adc[0]+5.4*adc[3]);//2.6   3.27   2.8
	
		//error = (+2.3*(sqrt(adc[1])-sqrt(adc[5])))/(adc[1]+adc[5]+2.1*adc[3]);		//不用斜电杆
	
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

		if (judge_sign == 1)		//最下面按键控制
		{		judge();	}
//		
	if(text==1 && motor_start==1)			//零漂结束
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
	

//		imu660ra_get_acc(); 		//获取加速度数据
//		imu660ra_get_gyro();    	//获取陀螺仪数据

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
	TIM2_CLEAR_FLAG;  //清除中断标志
	
}
void TM3_Isr() interrupt 19
{
	TIM3_CLEAR_FLAG; //清除中断标志
	
}
extern void pit_callback(void);
void TM4_Isr() interrupt 20
{
	TIM4_CLEAR_FLAG; //清除中断标志
//	ccd_collect();	 //CCD采集数据
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