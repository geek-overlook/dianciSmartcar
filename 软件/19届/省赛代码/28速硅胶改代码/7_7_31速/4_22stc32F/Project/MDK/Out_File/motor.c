#include "headfile.h" // 包含头文件，确保所有必要的函数和定义可用

// 编码器和目标速度的初始化
int r_encoder = 0; // 右电机编码器值
int l_encoder = 0; // 左电机编码器值
float r_target_speed = 28.0; // 右电机目标速度
float l_target_speed = 28.0; // 左电机目标速度

// 速度和误差变量
float r_speed = 0.0, l_speed = 0.0; // 右电机和左电机的实际速度
float r_endspeed = 0; // 右电机期望速度
float l_endspeed = 0; // 左电机期望速度
float l_error = 0; // 左电机误差
float r_error = 0; // 右电机误差

char turn_count = 0; // 转向计数器

// 上一个输入值
float last_in_L = 0; // 左电机上一个输入
float last_in_R = 0; // 右电机上一个输入
float T = 0.005; // 时间间隔

float last_in_turn = 0; // 上一个转向输入

// 常量定义
#define ENCODER_RESOLUTION 1024.0f // 编码器分辨率
#define SPEED_SCALING_FACTOR 200.0f // 速度缩放因子

void motor(void) // 电机控制函数
{
	// 读取编码器值
	l_encoder = ctimer_count_read(MOTOR1_ENCODER); // 读取左电机编码器
	r_encoder = ctimer_count_read(MOTOR2_ENCODER); // 读取右电机编码器
	
	// 清除计时器计数
	ctimer_count_clean(CTIM0_P34);
	ctimer_count_clean(CTIM3_P04);
	
	// 计算电机速度
	l_speed = calculate_motor_speed(l_encoder, MOTOR1_DIR); // 计算左电机速度
	r_speed = calculate_motor_speed(r_encoder, MOTOR2_DIR); // 计算右电机速度
	
	motor_inc += (fabs(r_speed) + fabs(l_speed)) * 0.5f; // 更新电机增量

	// 控制转向
	if (turn_count < 5)
	{
		turn_count++; // 增加转向计数
	}
	else if (turn_count >= 5)
	{
		turn_speed = forwardfeed_turn(turn_PstPID(error, &turn_pid)); // 计算转向速度
		turn_count = 0; // 重置转向计数
	}
	
	// 根据距离保护和速度控制电机
	if (distance_protect)
	{
		update_motor_speed(); // 更新电机速度
	}

	// 限制 PWM 值范围
	r_pwm = constrain_pwm(r_pwm);
	l_pwm = constrain_pwm(l_pwm);

	// 根据 PWM 值控制电机
	control_motor(MOTOR2, r_pwm, P2_4); // 控制右电机
	control_motor(MOTOR1, l_pwm, P2_6); // 控制左电机
}

// 计算电机速度的函数
float calculate_motor_speed(int encoder_value, int motor_direction)
{
	float speed = (float)encoder_value / ENCODER_RESOLUTION * SPEED_SCALING_FACTOR; // 计算速度
	return (motor_direction == 0) ? speed : -speed; // 根据方向返回速度
}

// 限制 PWM 值范围的函数
float constrain_pwm(float pwm_value)
{
	if (pwm_value >= 8000) return 8000;
	if (pwm_value < -8000) return -8000;
	return pwm_value;
}

// 控制电机的函数
void control_motor(int motor, float pwm_value, int gpio_pin)
{
	if (pwm_value >= 0)
	{
		gpio_mode(gpio_pin, GPO_PP); // 设置 GPIO 模式
		pwm_duty(motor, pwm_value); // 设置电机 PWM
	}
	else
	{
		gpio_mode(gpio_pin, GPO_PP); // 设置 GPIO 模式
		pwm_duty(motor, -pwm_value); // 设置电机 PWM
	}
}

// 更新电机速度的函数
void update_motor_speed()
{
	// 根据距离保护和速度控制电机的逻辑
	// 这里可以添加具体的速度更新逻辑
}




