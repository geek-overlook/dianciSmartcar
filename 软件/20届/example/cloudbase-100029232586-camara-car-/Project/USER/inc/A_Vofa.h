#ifndef __A_VOFA_H
#define __A_VOFA_H

#include "SEEKFREE_WIRELESS.h"//����ת����

#define  VOFA_MAX 255        //��������� 
#define  UART_PutChar(byte)		wireless_uart_send_buff( byte, 4);

/*
Ҫ����ʾ:
1. float��unsigned long������ͬ�����ݽṹ����
2. union������������ݴ������ͬ�������ռ�
*/
typedef union     
{
    float fdata;
    unsigned long ldata; //�ĸ��ֽ�
}FloatLongType1;


extern float vofa_send_data[VOFA_MAX];//��������
extern void vodka_JustFloat_send(float *data_str,uint16 num);
#endif
