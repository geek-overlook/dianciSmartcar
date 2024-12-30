#include "headfile.h"
#include "control.h"
#include "PID.h"



_PID Gyroy_PID,WPitch_PID,Speed_PID,Servo_PID,Go_Speed_PID;
_PID YGyrox_PID,Yaw_angle_PID,YSpeed_PID;
_PID TGyroz_PID,Turn_PID,TSpeed_PID;
_PID mr_pid,ml_pid;//���ҵ��pid
_PID R_pid,L_pid;

void PID_init(_PID* sptr)
{
    sptr->kp = 0.0;              // ��������
    sptr->ki = 0.0;              // ���ֳ���
    sptr->kd = 0.0;              // ΢�ֳ���
    sptr->kp1 = 0.0;             // ���ӱ������� 1�����ʹ�ã�
    sptr->kp2 = 0.0;             // ���ӱ������� 2�����ʹ�ã�

    sptr->limit_max = 0;         // ������ֵ����
    sptr->limit_min = 0;         // �����Сֵ����

    sptr->integral_max = 0.0;   // ���ֲ��ֵ����ֵ
    sptr->integral_min = 0.0;   // ���ֲ��ֵ���Сֵ

    sptr->err = 0.0;             // ��ǰ���
    sptr->err_sum = 0.0;         // ����ܺͣ����ֲ��֣�
    sptr->err_last = 0.0;        // �ϴ�������΢�ּ��㣩
    sptr->d_err = 0.0;           // ΢�������仯����

    sptr->out = 0.0;             // PID �����������ֵ
    sptr->integral_out = 0.0;    // ���ֲ��ֵ����
    sptr->kp_out = 0.0;          // �������ֵ����
    sptr->kd_out = 0.0;          // ΢�ֲ��ֵ����
    sptr->kd_lastout = 0.0;      // �ϴ�΢�ֲ��ֵ����

    sptr->Target = 0.0;          // Ŀ��ֵ������ֵ��
    sptr->now = 0.0;             // ��ǰ���̱���������ֵ��

    sptr->filter = 0.0;          // �����˲�ֵ�������Ҫ�Ļ���
}

// PID���ú���
void PID_Set(_PID *PID, float kp, float Ki, float Kd, 
             float kp1, float kp2, int Limit_Max, int Limit_Min,
             float Integral_Max, float Integral_Min, 
             float Target, float Now)
{
    // ����PID����
    PID->kp = kp;
    PID->ki = Ki;
    PID->kd = Kd;
    
    // ���ø��ӵı�������
    PID->kp1 = kp1;
    PID->kp2 = kp2;
    
    // ��������ֵ
    PID->limit_max = Limit_Max;
    PID->limit_min = Limit_Min;
    
    // ���û������ֵ����Сֵ
    PID->integral_max = Integral_Max;
    PID->integral_min = Integral_Min;
    
    // ��ʼ��������ֵ
    PID->err = 0.0;
    PID->err_sum = 0.0;
    PID->err_last = 0.0;
    PID->d_err = 0.0;
    
    // �������ֵ
    PID->out = 0.0;
    PID->integral_out = 0.0;
    PID->kp_out = 0.0;
    PID->kd_out = 0.0;
    PID->kd_lastout = 0.0;
    
    // ����Ŀ��ֵ�͵�ǰֵ
    PID->Target = Target;
    PID->now = Now;
    
    // ��ʼ���˲���ֵ
    PID->filter = 0.0;
}

// �ٶȻ�(����ʽPID)
float IncPID(float Encoder, float Target, _PID* sptr) 
{
    float Pwm;
    
    // ���㵱ǰ���
    sptr->err = Target - Encoder;  // ��ǰ��� = Ŀ��ֵ - ��ǰ����ֵ

    // ������ (P)
    sptr->kp_out = sptr->kp * sptr->err;  // ������ = kp * ��ǰ���
    
    // ������ (I)
    sptr->err_sum += sptr->err;  // �ۼӵ�ǰ����Ϊ������
    if (sptr->err_sum > sptr->integral_max) {
        sptr->err_sum = sptr->integral_max;  // ���ƻ������ֵ
    } else if (sptr->err_sum < sptr->integral_min) {
        sptr->err_sum = sptr->integral_min;  // ���ƻ�����Сֵ
    }
    sptr->integral_out = sptr->ki * sptr->err_sum;  // ������ = Ki * �������

    // ΢���� (D)
    sptr->d_err = sptr->err - sptr->err_last;  // �������仯��
    sptr->kd_out = sptr->kd * sptr->d_err;  // ΢���� = Kd * ���仯��

    // PID ������� = ������ + ������ + ΢����
    Pwm = sptr->kp_out + sptr->integral_out + sptr->kd_out;
    
    // �������
    if (Pwm > sptr->limit_max) {
        Pwm = sptr->limit_max;  // ����������ֵ
    } else if (Pwm < sptr->limit_min) {
        Pwm = sptr->limit_min;  // ������С���ֵ
    }

    // ������ʷ���
    sptr->err_last = sptr->err;  // �洢��ǰ����Ϊ�´μ���ġ��ϴ���

    return Pwm;  // ����PID���
}

// λ��ʽPID
float PstPID(float Angle, float Target, _PID* sptr)
{
    float Pwm;

    // ���㵱ǰ���
    sptr->err = Target - Angle;  // ��ǰ��� = Ŀ��ֵ - ��ǰ�Ƕȣ���ǰλ�ã�

    // �����������
    sptr->kp_out = sptr->kp * sptr->err;  // ��������� = kp * ��ǰ���

    // ������ֲ���
    sptr->err_sum += sptr->err;  // ������ = ����ۼ�
    if (sptr->err_sum > sptr->integral_max) {
        sptr->err_sum = sptr->integral_max;  // ���ֲ��ֵ��������
    } else if (sptr->err_sum < sptr->integral_min) {
        sptr->err_sum = sptr->integral_min;  // ���ֲ��ֵ���С����
    }
    sptr->integral_out = sptr->ki * sptr->err_sum;  // ��������� = Ki * �ۻ����

    // ����΢�ֲ���
    sptr->d_err = sptr->err - sptr->err_last;  // ΢����� = ��ǰ��� - �ϴ����
    sptr->kd_out = sptr->kd * sptr->d_err;  // ΢������� = Kd * ΢�����

    // ����PID���
    Pwm = sptr->kp_out + sptr->integral_out + sptr->kd_out;

    // �������
    if (Pwm > sptr->limit_max) {
        Pwm = sptr->limit_max;  // ����������
    } else if (Pwm < sptr->limit_min) {
        Pwm = sptr->limit_min;  // ������С���
    }

    // ������ʷ���
    sptr->err_last = sptr->err;  // �洢��ǰ��������һ�ε�΢�ּ���

    return Pwm;  // ����PID���
}



