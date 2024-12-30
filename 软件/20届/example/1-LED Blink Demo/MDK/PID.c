#include "headfile.h"
#include "control.h"
#include "PID.h"



_PID Gyroy_PID,WPitch_PID,Speed_PID,Servo_PID,Go_Speed_PID;
_PID YGyrox_PID,Yaw_angle_PID,YSpeed_PID;
_PID TGyroz_PID,Turn_PID,TSpeed_PID;
_PID mr_pid,ml_pid;//左右电机pid
_PID R_pid,L_pid;

void PID_init(_PID* sptr)
{
    sptr->kp = 0.0;              // 比例常数
    sptr->ki = 0.0;              // 积分常数
    sptr->kd = 0.0;              // 微分常数
    sptr->kp1 = 0.0;             // 附加比例常数 1（如果使用）
    sptr->kp2 = 0.0;             // 附加比例常数 2（如果使用）

    sptr->limit_max = 0;         // 输出最大值限制
    sptr->limit_min = 0;         // 输出最小值限制

    sptr->integral_max = 0.0;   // 积分部分的最大值
    sptr->integral_min = 0.0;   // 积分部分的最小值

    sptr->err = 0.0;             // 当前误差
    sptr->err_sum = 0.0;         // 误差总和（积分部分）
    sptr->err_last = 0.0;        // 上次误差（用于微分计算）
    sptr->d_err = 0.0;           // 微分误差（误差变化量）

    sptr->out = 0.0;             // PID 控制器的输出值
    sptr->integral_out = 0.0;    // 积分部分的输出
    sptr->kp_out = 0.0;          // 比例部分的输出
    sptr->kd_out = 0.0;          // 微分部分的输出
    sptr->kd_lastout = 0.0;      // 上次微分部分的输出

    sptr->Target = 0.0;          // 目标值（期望值）
    sptr->now = 0.0;             // 当前过程变量（测量值）

    sptr->filter = 0.0;          // 噪声滤波值（如果需要的话）
}

// PID设置函数
void PID_Set(_PID *PID, float kp, float Ki, float Kd, 
             float kp1, float kp2, int Limit_Max, int Limit_Min,
             float Integral_Max, float Integral_Min, 
             float Target, float Now)
{
    // 设置PID参数
    PID->kp = kp;
    PID->ki = Ki;
    PID->kd = Kd;
    
    // 设置附加的比例常数
    PID->kp1 = kp1;
    PID->kp2 = kp2;
    
    // 设置限制值
    PID->limit_max = Limit_Max;
    PID->limit_min = Limit_Min;
    
    // 设置积分最大值和最小值
    PID->integral_max = Integral_Max;
    PID->integral_min = Integral_Min;
    
    // 初始化误差相关值
    PID->err = 0.0;
    PID->err_sum = 0.0;
    PID->err_last = 0.0;
    PID->d_err = 0.0;
    
    // 设置输出值
    PID->out = 0.0;
    PID->integral_out = 0.0;
    PID->kp_out = 0.0;
    PID->kd_out = 0.0;
    PID->kd_lastout = 0.0;
    
    // 设置目标值和当前值
    PID->Target = Target;
    PID->now = Now;
    
    // 初始化滤波器值
    PID->filter = 0.0;
}

// 速度环(增量式PID)
float IncPID(float Encoder, float Target, _PID* sptr) 
{
    float Pwm;
    
    // 计算当前误差
    sptr->err = Target - Encoder;  // 当前误差 = 目标值 - 当前测量值

    // 比例项 (P)
    sptr->kp_out = sptr->kp * sptr->err;  // 比例项 = kp * 当前误差
    
    // 积分项 (I)
    sptr->err_sum += sptr->err;  // 累加当前误差，作为积分项
    if (sptr->err_sum > sptr->integral_max) {
        sptr->err_sum = sptr->integral_max;  // 限制积分最大值
    } else if (sptr->err_sum < sptr->integral_min) {
        sptr->err_sum = sptr->integral_min;  // 限制积分最小值
    }
    sptr->integral_out = sptr->ki * sptr->err_sum;  // 积分项 = Ki * 积分误差

    // 微分项 (D)
    sptr->d_err = sptr->err - sptr->err_last;  // 计算误差变化量
    sptr->kd_out = sptr->kd * sptr->d_err;  // 微分项 = Kd * 误差变化量

    // PID 控制输出 = 比例项 + 积分项 + 微分项
    Pwm = sptr->kp_out + sptr->integral_out + sptr->kd_out;
    
    // 输出限制
    if (Pwm > sptr->limit_max) {
        Pwm = sptr->limit_max;  // 限制最大输出值
    } else if (Pwm < sptr->limit_min) {
        Pwm = sptr->limit_min;  // 限制最小输出值
    }

    // 更新历史误差
    sptr->err_last = sptr->err;  // 存储当前误差，作为下次计算的“上次误差”

    return Pwm;  // 返回PID输出
}

// 位置式PID
float PstPID(float Angle, float Target, _PID* sptr)
{
    float Pwm;

    // 计算当前误差
    sptr->err = Target - Angle;  // 当前误差 = 目标值 - 当前角度（或当前位置）

    // 计算比例部分
    sptr->kp_out = sptr->kp * sptr->err;  // 比例项输出 = kp * 当前误差

    // 计算积分部分
    sptr->err_sum += sptr->err;  // 积分项 = 误差累计
    if (sptr->err_sum > sptr->integral_max) {
        sptr->err_sum = sptr->integral_max;  // 积分部分的最大限制
    } else if (sptr->err_sum < sptr->integral_min) {
        sptr->err_sum = sptr->integral_min;  // 积分部分的最小限制
    }
    sptr->integral_out = sptr->ki * sptr->err_sum;  // 积分项输出 = Ki * 累积误差

    // 计算微分部分
    sptr->d_err = sptr->err - sptr->err_last;  // 微分误差 = 当前误差 - 上次误差
    sptr->kd_out = sptr->kd * sptr->d_err;  // 微分项输出 = Kd * 微分误差

    // 计算PID输出
    Pwm = sptr->kp_out + sptr->integral_out + sptr->kd_out;

    // 输出限制
    if (Pwm > sptr->limit_max) {
        Pwm = sptr->limit_max;  // 限制最大输出
    } else if (Pwm < sptr->limit_min) {
        Pwm = sptr->limit_min;  // 限制最小输出
    }

    // 更新历史误差
    sptr->err_last = sptr->err;  // 存储当前误差，用于下一次的微分计算

    return Pwm;  // 返回PID输出
}



