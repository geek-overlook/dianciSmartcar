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
 
// *************************** ����ʹ�ò���˵�� ***************************
// 1.����Ӳ������˵�����Ӻ�ģ�飬ʹ�õ�Դ����(����������ᵼ��ģ���ѹ����)
//
// 2.�������̵���Ƭ���У�����ɴ������֡�
//
// 3.����ɴ��������У�ѡ��ʾ������
//
// 4.ѡ����������Ӧ�Ĵ��ںţ�������(Ĭ��115200)���������
//
// 5.ʾ���������̿�������

// *************************** ���̲���˵�� ***************************
// 1.�����̻�ͨ�� Debug �������������Ϣ ����ؽӺõ��Դ����Ա��ȡ������Ϣ
//
// 2.���Ӻ�ģ��ͺ��İ�󣨾���ʹ��������������Ա��⹩�粻������⣩ ��¼������ ���¸�λ�����ʼ����
//




//-------------------------------------------------------------------------------------------------------------------
// �������     �δ�ͷ��ͺ���
// ����˵��     *buff           ��Ҫ���͵����ݵ�ַ
// ����˵��     length          ��Ҫ���͵ĳ���
// ���ز���     uint32          ʣ��δ�������ݳ���
// ʹ��ʾ��
//-------------------------------------------------------------------------------------------------------------------
uint32 seekfree_assistant_transfer_callback   (const uint8 *buff, uint32 length)
{
	uart_putbuff(DEBUG_UART, buff, length);
	return 0;
}

void main()
{
	board_init();			// ��ʼ���Ĵ���,��ɾ���˾���롣
	
	// ���ú���ָ��
	seekfree_assistant_transfer = seekfree_assistant_transfer_callback;

	
	// �˴���д�û�����(���磺�����ʼ�������)
	
	// ��������
	seekfree_assistant_oscilloscope_data.dat[0] = 1;
	seekfree_assistant_oscilloscope_data.dat[1] = 2;
	seekfree_assistant_oscilloscope_data.dat[2] = 3.00;
	seekfree_assistant_oscilloscope_data.dat[3] = 3.22;
	// ��Ҫ�����ĸ�ͨ������
	seekfree_assistant_oscilloscope_data.channel_num = 4;
	
    while(1)
	{
		seekfree_assistant_oscilloscope_data.dat[0]++;
		seekfree_assistant_oscilloscope_data.dat[1]++;
		seekfree_assistant_oscilloscope_data.dat[2] += 3;
		seekfree_assistant_oscilloscope_data.dat[3] += 4;
		
		// ͨ�����ڷ�������ʾ��������
		seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
		
		delay_ms(1);
		// �˴���д��Ҫѭ��ִ�еĴ���
    }
}




