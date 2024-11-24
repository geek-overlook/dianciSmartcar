
#include "A_Vofa.h"

float vofa_send_data[VOFA_MAX] = {0};

// 把浮点数据转为4个八位数据，存到数组中
static void Float_to_Byte(float f, uint8 byte[]) 
{
    FloatLongType1 fl;
    fl.fdata = f;
    byte[0] = (unsigned char)fl.ldata;
    byte[1] = (unsigned char)(fl.ldata >> 8);
    byte[2] = (unsigned char)(fl.ldata >> 16);
    byte[3] = (unsigned char)(fl.ldata >> 24);
}

// vofa+上位机协议-Justfloat
void vodka_JustFloat_send(float *data_str,uint16 num) 
{
    uint8 i = 0;
    uint8 byte[4] = {0};
    uint8 tail[4] = {0x00, 0x00, 0x80, 0x7f};	
    // 循环遍历数据并发送
    for (i = 0; i < num; i++) 
    {
        Float_to_Byte(data_str[i], byte);
        UART_PutChar(byte);
    }
    
    UART_PutChar(tail);// 发送帧尾
}


