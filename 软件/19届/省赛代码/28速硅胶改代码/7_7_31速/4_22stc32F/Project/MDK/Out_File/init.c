#include "headfile.h"

// ADC 读取值数组
float adc[11] = {0}; // 用于存储 ADC 读取的值

// 初始化陀螺仪的变量
int init_gyro_x = 0, init_gyro_y = 0, init_gyro_z = 0; // 陀螺仪初始化值
float new_gyro_x = 0, new_gyro_y = 0, new_gyro_z = 0; // 新的陀螺仪值
float z_inc = 0, x_inc = 0, y_inc = 0; // 增量值
float angle_x; // 角度值

// 上一个加速度和陀螺仪值
float last_acc_x, last_acc_y, last_acc_z; 
float last_gyro_x, last_gyro_y, last_gyro_z;

// 当前加速度和陀螺仪值
float Xdata = 0.0, Ydata = 0.0, Zdata = 0.0; 
float acc_x = 0.0, acc_y = 0.0, acc_z = 0.0; 
float gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0; 

int text = 0; // 文本标志位
float alpha = 0.3; // 平滑因子

char a = 0; // 计数器

// PWM 变量
float r_pwm = 0.0, l_pwm = 0.0; // 右电机和左电机的 PWM 值
float last_r_pwm = 0.0, last_l_pwm = 0.0; // 上一个 PWM 值

#define ADC_CHANNELS 7 // ADC 通道数量
const int adc_channels[ADC_CHANNELS] = {ADC0, ADC1, ADC2, ADC3, ADC4, ADC5, ADC6}; // ADC 通道数组

void car_pre_init(void) // 车辆预初始化函数
{
	// 初始化 PID 控制器
	PID_init(&mr_pid); // 右电机 PID
	PID_init(&ml_pid); // 左电机 PID
	PID_init(&R_pid); // 右轮 PID
	PID_init(&L_pid); // 左轮 PID
	PID_init(&turn_pid); // 转向 PID

	// 设置 PID 参数
	PID_Set(&R_pid, 200, 25, 17); // 设置右轮 PID 参数
	PID_Set(&L_pid, 200, 25, 17); // 设置左轮 PID 参数
	PID_Set(&w_pid, 0.7, 0.1, 0); // 设置速度 PID 参数
	PID_turnSet(&turn_pid, 190, 5, 17, 24); // 设置转向 PID 参数

	// 初始化 ADC
	for (int i = 0; i < ADC_CHANNELS; i++) {
		adc_init(adc_channels[i], ADC_SYSclk_DIV_2); // 初始化每个 ADC 通道
	}
	delay_ms(10); // 延迟以确保初始化完成
	
	// 初始化 PWM
	pwm_init(MOTOR1, 15000, 0); // 初始化电机1 PWM
	pwm_init(MOTOR2, 15000, 0); // 初始化电机2 PWM
	delay_ms(10); // 延迟以确保初始化完成
	
	// 初始化 IPS 显示器
	ips114_init(); // 初始化 IPS 显示器
	
	// 初始化计时器
	ctimer_count_init(MOTOR1_ENCODER); // 初始化电机1编码器
	ctimer_count_init(MOTOR2_ENCODER); // 初始化电机2编码器
	delay_ms(30); // 延迟以确保初始化完成
	
	// 初始化 Lidar 传感器
	Lidar1.Address = 0x10; // 设置 Lidar 地址
	User_I2C_Init(); // 初始化 I2C
	imu660ra_init(); // 初始化 IMU 传感器
	target_speed_save = l_target_speed; // 保存目标速度

	return; // 返回
}

void get_adc(void) // 获取 ADC 值
{
	int adc_get_value[ADC_CHANNELS] = {0}; // 存储 ADC 读取值的数组
	for (int i = 0; i < 5; ++i) // 读取 5 次 ADC 值
	{
		for (int j = 0; j < ADC_CHANNELS; ++j) {
			adc_get_value[j] += adc_once(adc_channels[j], ADC_10BIT); // 读取每个 ADC 通道
		}
	}
	for (int k = 0; k < ADC_CHANNELS; ++k) // 计算平均值
	{
		adc[k] = (float)adc_get_value[k] / (5 * 1000.0f); // 将 ADC 值转换为浮点数并存储
	}
}

// 陀螺仪偏移初始化
void gyroOffsetInit(void) 
{
	if (text == 0) // 如果文本标志为 0
	{
		if (a < 100) // 计数器小于 100
		{
			imu660ra_get_acc(); // 获取加速度
			imu660ra_get_gyro(); // 获取陀螺仪数据
			Xdata += imu660ra_gyro_x; // 累加陀螺仪 X 轴数据
			delay_ms(5); // 延迟 5 毫秒
			a++; // 计数器加 1
		}
		else if (a == 100) // 当计数器等于 100
		{
			Xdata *= 0.01; // 计算平均值
			ips_page = 1; // 重置 IPS 页面
			a++; // 计数器加 1
			text = 1; // 更新文本标志
		}
	}
	else if (text == 1) // 如果文本标志为 1
	{
		imu660ra_get_acc(); // 获取加速度
		imu660ra_get_gyro(); // 获取陀螺仪数据
		gyro_x = imu660ra_gyro_transition(imu660ra_gyro_x - Xdata) * alpha + last_gyro_x * (1 - alpha); // 计算平滑的陀螺仪 X 轴值
		last_gyro_x = gyro_x; // 更新上一个陀螺仪 X 轴值
	}
}

