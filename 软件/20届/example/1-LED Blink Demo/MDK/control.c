#include "headfile.h"
#include "control.h"


//adc��ȡֵ
float adc[11]={0} ;

//IMU�Լ�״̬
int IMU_state=100;

int Key_testNum=100;


void car_init(void){
			
			//lcd��ʼ��
			lcd_init();  
			lcd_clear(WHITE); 
			
			//����������������
			gpio_pull_set(P70,PULLUP);
			gpio_pull_set(P71,PULLUP);
			gpio_pull_set(P72,PULLUP);
			gpio_pull_set(P73,PULLUP);
			gpio_pull_set(P75,PULLUP);
			
			//���뿪�ؿ�����Ļ���
			gpio_pull_set(P37,PULLUP);
	
			//��������ʼ��
			ctimer_count_init(MOTOR1_ENCODER);
			ctimer_count_init(MOTOR2_ENCODER);
			delay_ms(30);
			
			//PWM��ʼ��
			pwm_init(MOTOR1, 15000, 0);
			pwm_init(MOTOR2, 15000, 0);
			delay_ms(10);
			
			//��ʼ��ADC
			adc_init(ADC_P01, ADC_SYSclk_DIV_2);	//��ʼ��ADC,P0.6ͨ�� ��ADCʱ��Ƶ�ʣ�SYSclk/2
			adc_init(ADC_P00, ADC_SYSclk_DIV_2);	//��ʼ��ADC,P1.1ͨ�� ��ADCʱ��Ƶ�ʣ�SYSclk/2
			adc_init(ADC_P02, ADC_SYSclk_DIV_2);	//��ʼ��ADC,P1.4ͨ�� ��ADCʱ��Ƶ�ʣ�SYSclk/2
			adc_init(ADC_P15, ADC_SYSclk_DIV_2);	//��ʼ��ADC,P1.5ͨ�� ��ADCʱ��Ƶ�ʣ�SYSclk/2
			adc_init(ADC_P14, ADC_SYSclk_DIV_2);	//��ʼ��ADC,P0.6ͨ�� ��ADCʱ��Ƶ�ʣ�SYSclk/2
			adc_init(ADC_P11, ADC_SYSclk_DIV_2);	//��ʼ��ADC,P1.1ͨ�� ��ADCʱ��Ƶ�ʣ�SYSclk/2
			adc_init(ADC_P05, ADC_SYSclk_DIV_2);	//��ʼ��ADC,P1.4ͨ�� ��ADCʱ��Ƶ�ʣ�SYSclk/2
			adc_init(ADC_P06, ADC_SYSclk_DIV_2);	//��ʼ��ADC,P1.5ͨ�� ��ADCʱ��Ƶ�ʣ�SYSclk/2
			delay_ms(10);
			//�����ǳ�ʼ��
			imu660ra_init();
			
			
			
			//������ʼ��
			PID_init(&mr_pid);
			PID_init(&ml_pid);
			
			PID_init(&R_pid);  			//�ٶȻ�
			PID_init(&L_pid);
			
			PID_init(&Turn_PID);
			
}



/*
����������
mode:
���ư���״̬�Ƿ����á�
mode = 1: ���ð���״̬Ϊδ���¡�
mode = 0: ������ⰴ��״̬
*/
uint8 key_scan(int mode) {
			
    static unsigned int key = 1; // ����״̬��1��ʾδ���£�0��ʾ�Ѱ���

    if (mode)key = 1; // ���ģʽΪ�棬���ð���״̬Ϊδ����

    // ��ⰴ������
    if (key == 1 && (P70 == 0 || P71 == 0 || P72 == 0 || P73 == 0 || P75 == 0)) {
        delay_ms(10); // ��������
        key = 0; // ����Ϊ�Ѱ���״̬

        // �������ĸ�����������
        if (P70 == 0) return 1; // ����1��ʾ����P7_0
        else if (P71 == 0) return 2; // ����2��ʾ����P7_1
        else if (P72 == 0) return 3; // ����3��ʾ����P7_2
        else if (P73 == 0) return 4; // ����4��ʾ����P7_3
        else if (P75 == 0) return 5; // ����5��ʾ����P7_5
    }
    // ������а����ͷ�
    else if (P70 == 1 && P71 == 1 && P72 == 1 && P73 == 1 && P75 == 1) {
        key = 1; // ����Ϊδ����״̬
    }

    return 0; // û�а���������
					
}

/*
*ADC��ȡ������
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

/*�����뿪�ش��ϣ���Ļ�˵��ر�����
*
*/
void Screen_Control(void) {
    if(Screen_switch==0)
		{		key_process();
		lcd_choice_function();}
}