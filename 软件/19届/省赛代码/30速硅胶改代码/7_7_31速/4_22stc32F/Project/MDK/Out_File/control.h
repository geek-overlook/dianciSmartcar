#ifndef _CONTROL_H_
#define _CONTROL_H_

typedef struct 
{
		 float   Kp;             // �������� Proportional Const
		 float   Ki;               // ���ֳ��� Integral Const
		 float   Kd;             // ΢�ֳ��� Derivative Const
		 float 	 Kd2;
		 float			Integral;
		 float      Bias;                 
		 float      Last_Bias;                
		 float      Pre_Bias;                
}PID_TypDef;

extern PID_TypDef mr_pid,ml_pid,Steer_pid,turn_pid,w_pid;		//���pid	
extern PID_TypDef s_pid,R_pid,L_pid;		//���pid




void PID_init(PID_TypDef* sptr);   //===PID��ֵ��ʼ��
void PID_Set(PID_TypDef *PID,float Kp,float Ki,float Kd);   //===PID��ֵ����
void PID_turnSet(PID_TypDef *PID,float Kp,float Ki,float Kd,float Kd2);


float IncPID(float Encoder,float Target,PID_TypDef* sptr);    	//===����ʽpid
float PstPID(float Angle, float Target,PID_TypDef* sptr);   //===λ��ʽPId
float turn_PstPID(float turn_error,PID_TypDef* sptr);				//ת������ʽPID





#endif
