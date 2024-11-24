#include "headfile.h"



// 关于内核频率的设定，可以查看board.h文件
// 在board_init中,已经将P54引脚设置为复位
// 如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可
uint16 last_ccd_data[128];
float offset2_weight=8;




void main()
{
	uint16 count_time=0;
	uint8 display_mode=0;
	clock_init(SYSTEM_CLOCK_52M);	// 初始化系统频率,勿删除此句代码。
	board_init();					// 初始化寄存器,勿删除此句代码。				// 初始化寄存器,勿删除此句代码。
	//DisableGlobalIRQ();
	
	//车辆初始化
	car_init();
	
	//ccd、tof位置检查
	car_check();

	while(1){

		// vofa_send_data[0] = 0;
		// vofa_send_data[1] = 0;
		// vofa_send_data[2] = 0;
		// vofa_send_data[7] = 0;
		// vodka_JustFloat_send(vofa_send_data, 8);


		key_test=key_scan(0);





		if(key_test==KEY2_PRESS)car_ready_gogogo = 1;


		
 		if(tsl1401_finish_flag){  //判断完成一次计算就刷新lcd
			if(key_test==KEY1_PRESS){
				display_mode += 1;
				lcd_clear(BLACK); //  清屏
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
			tsl1401_finish_flag = 0; //标志位清零
		}
	}
}


