#ifndef _CONTROL_H
#define _CONTROL_H
#include "headfile.h"
//�������ӿڶ���
#define MOTOR1_ENCODER CTIM0_P34			//Z��
#define MOTOR2_ENCODER CTIM3_P04
#define MOTOR1_DIR P35								//dir����
#define MOTOR2_DIR P53


//����ӿڶ���
#define MOTOR1 PWMA_CH2N_P13 //��   8701
#define MOTOR2 PWMA_CH1P_P10 //��
#define MOTOR1_d P2_6
#define MOTOR2_d P2_4


//ADC
#define ADC0 ADC_P05//ADC_P01
#define ADC1 ADC_P11//ADC_P00
#define ADC2 ADC_P00//ADC_P02
#define ADC3 ADC_P14//ADC_P15
#define ADC4 ADC_P01//ADC_P14	
#define ADC5 ADC_P15//ADC_P11
#define ADC6 ADC_P02//ADC_P05	

//����
#define upPin P75
#define downPin P70
#define leftPin P73
#define rightPin P72
#define middlePin P71

#define Screen_switch P37

void car_init();
uint8 key_scan(int mode);
void get_adc(void);				//ADC��ȡ������
void Screen_Control(void);//tft��Ļ����

extern int Key_testNum;


#endif 

