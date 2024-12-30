#include "headfile.h"
#include "Motor.h"

// 编码器值
uint16 r_encoder = 0;
uint16 l_encoder = 0;

// 目标速度
float r_target_speed = 20.0;
float l_target_speed = 20.0;

// 累计电机增量
float motor_inc = 0;

// 实际速度
float r_speed = 0.0, l_speed = 0.0;

// 目标转速
float r_endspeed = 0;
float l_endspeed = 0;

// 速度误差
float l_error = 0;
float r_error = 0;

// 实际转向
float set_turn = 0.37;

// 上一次的输入值
float last_in_L = 0;
float last_in_R = 0;
float last_in_turn = 0;

// 时间间隔
float T = 0.005;

// 前进标志
float forward_flag = 0.17; // 前进前馈控制的权重

// 距离保护标志
char distance_protect = 0; // 1 表示开启，0 表示关闭
extern char flag_distance_protect; // 外部距离保护标志

// 角度速度和转向速度
float angle_speed = 0;
float turn_speed = 0;

// 文本标志位
int text = 0; // 文本标志位

// PWM 值
float r_pwm = 0.0;
float l_pwm = 0.0;

// 计算速度
float calculate_speed(uint16 encoder_value, int direction) {
    return (float)encoder_value / 1024.0f * 200.0f * (direction == 0 ? 1.0f : -1.0f);
}

// 电机控制函数
void motor(void)
{
    // 读取编码器值
    l_encoder = ctimer_count_read(MOTOR1_ENCODER);
    r_encoder = ctimer_count_read(MOTOR2_ENCODER);
    
    // 清除编码器计数
    ctimer_count_clean(CTIM0_P34);
    ctimer_count_clean(CTIM3_P04);
    
    // 计算左电机速度
    l_speed = calculate_speed(l_encoder, MOTOR1_DIR);

    // 计算右电机速度
    r_speed = calculate_speed(r_encoder, MOTOR2_DIR);
    
    // 累加电机增量
    motor_inc += (fabs(r_speed) + fabs(l_speed)) * 0.5f;
    
    // 控制 PWM 基于距离保护和角度/转向速度

    //TESTTESTTESTTESTTESTTESTTEST
}

// 左电机前馈控制
float forwardfeed_L(float inc_in) {
    float inc_out;
    inc_out = (inc_in - last_in_L) * forward_flag + inc_in;
    last_in_L = inc_in;
    return inc_out;
}

// 右电机前馈控制
float forwardfeed_R(float inc_in) {
    float inc_out;
    inc_out = (inc_in - last_in_R) * forward_flag + inc_in;
    last_in_R = inc_in;
    return inc_out;
}

// 转向前馈控制
float forwardfeed_turn(float inc_in) {
    float inc_out;
    inc_out = (inc_in - last_in_turn) * 0.15 + inc_in;
    last_in_turn = inc_in;
    return inc_out;
}