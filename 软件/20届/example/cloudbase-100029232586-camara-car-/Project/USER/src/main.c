#include "headfile.h"



// �����ں�Ƶ�ʵ��趨�����Բ鿴board.h�ļ�
// ��board_init��,�Ѿ���P54��������Ϊ��λ
// �����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����
uint16 last_ccd_data[128];
float offset2_weight=8;




void main()
{
	uint16 count_time=0;
	uint8 display_mode=0;
	clock_init(SYSTEM_CLOCK_52M);	// ��ʼ��ϵͳƵ��,��ɾ���˾���롣
	board_init();					// ��ʼ���Ĵ���,��ɾ���˾���롣				// ��ʼ���Ĵ���,��ɾ���˾���롣
	//DisableGlobalIRQ();
	
	//������ʼ��
	car_init();
	
	//ccd��tofλ�ü��
	car_check();

	while(1){

		// vofa_send_data[0] = 0;
		// vofa_send_data[1] = 0;
		// vofa_send_data[2] = 0;
		// vofa_send_data[7] = 0;
		// vodka_JustFloat_send(vofa_send_data, 8);


		key_test=key_scan(0);





		if(key_test==KEY2_PRESS)car_ready_gogogo = 1;


		
 		if(tsl1401_finish_flag){  //�ж����һ�μ����ˢ��lcd
			if(key_test==KEY1_PRESS){
				display_mode += 1;
				lcd_clear(BLACK); //  ����
				if(display_mode >= 2)display_mode = 0;
				}
			switch (display_mode){
				case 0 :{
					display_0();
					break;
				}
				case 1 :{
					display_1();
					break;
				}
			}
			tsl1401_finish_flag = 0; //��־λ����
		}
	}
}


