#include "headfile.h"
#include "control.h"


//adc获取值
float adc[11]={0} ;

//IMU自检状态
int IMU_state=100;

int Key_testNum=100;


void car_init(void){
			
			//lcd初始化
			lcd_init();  
			lcd_clear(WHITE); 
			
			//按键配置上拉电阻
			gpio_pull_set(P70,PULLUP);
			gpio_pull_set(P71,PULLUP);
			gpio_pull_set(P72,PULLUP);
			gpio_pull_set(P73,PULLUP);
			gpio_pull_set(P75,PULLUP);
			
			//拨码开关控制屏幕输出
			gpio_pull_set(P37,PULLUP);
	
			//编码器初始化
			ctimer_count_init(MOTOR1_ENCODER);
			ctimer_count_init(MOTOR2_ENCODER);
			delay_ms(30);
			
			//PWM初始化
			pwm_init(MOTOR1, 15000, 0);
			pwm_init(MOTOR2, 15000, 0);
			delay_ms(10);
			
			//初始化ADC
			adc_init(ADC_P01, ADC_SYSclk_DIV_2);	//初始化ADC,P0.6通道 ，ADC时钟频率：SYSclk/2
			adc_init(ADC_P00, ADC_SYSclk_DIV_2);	//初始化ADC,P1.1通道 ，ADC时钟频率：SYSclk/2
			adc_init(ADC_P02, ADC_SYSclk_DIV_2);	//初始化ADC,P1.4通道 ，ADC时钟频率：SYSclk/2
			adc_init(ADC_P15, ADC_SYSclk_DIV_2);	//初始化ADC,P1.5通道 ，ADC时钟频率：SYSclk/2
			adc_init(ADC_P14, ADC_SYSclk_DIV_2);	//初始化ADC,P0.6通道 ，ADC时钟频率：SYSclk/2
			adc_init(ADC_P11, ADC_SYSclk_DIV_2);	//初始化ADC,P1.1通道 ，ADC时钟频率：SYSclk/2
			adc_init(ADC_P05, ADC_SYSclk_DIV_2);	//初始化ADC,P1.4通道 ，ADC时钟频率：SYSclk/2
			adc_init(ADC_P06, ADC_SYSclk_DIV_2);	//初始化ADC,P1.5通道 ，ADC时钟频率：SYSclk/2
			delay_ms(10);
			//陀螺仪初始化
			imu660ra_init();
			
			
			
			//抽象层初始化
			PID_init(&mr_pid);
			PID_init(&ml_pid);
			
			PID_init(&R_pid);  			//速度环
			PID_init(&L_pid);
			
			PID_init(&Turn_PID);
			
}



/*
按键检测程序
mode:
控制按键状态是否重置。
mode = 1: 重置按键状态为未按下。
mode = 0: 正常检测按键状态
*/
uint8 key_scan(int mode) {
			
    static unsigned int key = 1; // 按键状态，1表示未按下，0表示已按下

    if (mode)key = 1; // 如果模式为真，重置按键状态为未按下

    // 检测按键按下
    if (key == 1 && (P70 == 0 || P71 == 0 || P72 == 0 || P73 == 0 || P75 == 0)) {
        delay_ms(10); // 消抖处理
        key = 0; // 设置为已按下状态

        // 检测具体哪个按键被按下
        if (P70 == 0) return 1; // 返回1表示按下P7_0
        else if (P71 == 0) return 2; // 返回2表示按下P7_1
        else if (P72 == 0) return 3; // 返回3表示按下P7_2
        else if (P73 == 0) return 4; // 返回4表示按下P7_3
        else if (P75 == 0) return 5; // 返回5表示按下P7_5
    }
    // 检测所有按键释放
    else if (P70 == 1 && P71 == 1 && P72 == 1 && P73 == 1 && P75 == 1) {
        key = 1; // 重置为未按下状态
    }

    return 0; // 没有按键被按下
					
}

/*
*ADC获取并处理
*/

void get_adc(void)	
{

	int i = 0 ,k = 0;
	int adc_get_value[7] = {0};	

	for( i = 0;i < 5;++i)
	{
		adc_get_value[0] += adc_once(ADC0,ADC_10BIT);
		adc_get_value[1] += adc_once(ADC1,ADC_10BIT);
		adc_get_value[2] += adc_once(ADC2,ADC_10BIT);
		adc_get_value[3] += adc_once(ADC3,ADC_10BIT);
		adc_get_value[4] += adc_once(ADC4,ADC_10BIT);//5
		adc_get_value[5] += adc_once(ADC5,ADC_10BIT);//6
		adc_get_value[6] += adc_once(ADC6,ADC_10BIT);//7	
	}
	for( k = 0;k < 7;++k)
	 {
		 adc[k] = (float)adc_get_value[k] / (5*1000.0f);
	 }
}

//

/*当拨码开关打上，屏幕菜单关闭运行
*
*/
void Screen_Control(void) {
    if(Screen_switch==0)
		{		key_process();
		lcd_choice_function();}
}