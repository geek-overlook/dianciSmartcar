
#ifndef __PID_H_
#define __PID_H_

typedef struct{
       float kp;
       float ki;
       float kd;
       float kp1;
       float kp2;

       int limit_max;
       int limit_min;

       float integral_max;
       float integral_min;

       float err;
       float err_sum;
       float err_last;
       float d_err;

       float out;
       float integral_out;
       float kp_out;
       float kd_out;
       float kd_lastout;
       float Target;
       float now;

       float filter;
}_PID;

extern _PID Gyroy_PID,WPitch_PID,Speed_PID,Servo_PID,YGyrox_PID,Yaw_angle_PID,Go_Speed_PID,Turn_PID;
extern _PID mr_pid,ml_pid;//×óÓÒµç»úpid
extern _PID R_pid,L_pid;


void PID_init(_PID* sptr);
void PID_Set(_PID *PID, float Kp, float Ki, float Kd, 
             float Kp1, float Kp2, int Limit_Max, int Limit_Min,
             float Integral_Max, float Integral_Min, 
             float Target, float Now);
float IncPID(float Encoder, float Target, _PID* sptr);
float PstPID(float Angle, float Target, _PID* sptr);



#endif