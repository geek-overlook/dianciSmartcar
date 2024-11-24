/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-12-18
 ********************************************************************************************************************/

#include "headfile.h"


/*
 * ϵͳƵ�ʣ��ɲ鿴board.h�е� FOSC �궨���޸ġ�
 * board.h�ļ���FOSC��ֵ����Ϊ0,������Զ�����ϵͳƵ��Ϊ33.1776MHZ
 * ��board_init��,�Ѿ���P54��������Ϊ��λ
 * �����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����
 */

uint16 adc_data[3];
void main()
{
	board_init();			// ��ʼ���Ĵ���,��ɾ���˾���롣
	
	// �˴���д�û�����(���磺�����ʼ�������)
	
    adc_init(ADC_P10, ADC_SYSclk_DIV_2);	//��ʼ��ADC,P1.0ͨ�� ��ADCʱ��Ƶ�ʣ�SYSclk/2
    adc_init(ADC_P11, ADC_SYSclk_DIV_2);	//��ʼ��ADC,P1.1ͨ�� ��ADCʱ��Ƶ�ʣ�SYSclk/2
    adc_init(ADC_P13, ADC_SYSclk_DIV_2);	//��ʼ��ADC,P1.2ͨ�� ��ADCʱ��Ƶ�ʣ�SYSclk/2

    while(1)
	{
		//ʹ�����ߵ��ԣ��鿴adc_data�������ֵ�����Եõ�AD���ݡ�
		adc_data[0] = adc_once(ADC_P10, ADC_12BIT);	//�ɼ�һ��ADC������10λ
		adc_data[1] = adc_once(ADC_P11, ADC_10BIT);	//�ɼ�һ��ADC������9λ
		adc_data[2] = adc_once(ADC_P13, ADC_8BIT);	//�ɼ�һ��ADC������8λ
		printf("adc_data[0] = %d\r\n", adc_data[0]);
		printf("adc_data[1] = %d\r\n", adc_data[1]);
		printf("adc_data[2] = %d\r\n", adc_data[2]);
		
		delay_ms(100);
    }
}



