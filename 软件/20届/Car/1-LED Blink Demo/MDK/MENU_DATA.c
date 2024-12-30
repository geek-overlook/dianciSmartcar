#include "headfile.h"
#include "control.h"
#include "PID.h"
#include "MENU_DATA.h"
#include "Menu.h"

int flash[20]={0};
uint16 address = 0x00; // ÆðÊ¼µØÖ·

void read_flash(void)
{
			int i;
			for(i=0;i<20;i++){
					iap_read_bytes(address,(uint8 *)&flash[i],4);
					address+=4;
			}
			start_speed=flash[0];
			wifi_flag=flash[1];
	
}

void write_flash(void)
{

					iap_erase_page(0);
					iap_write_bytes(0x00, (uint8*)&start_speed, 4);
					iap_write_bytes(0x04, (uint8*)&wifi_flag, 4);


}


