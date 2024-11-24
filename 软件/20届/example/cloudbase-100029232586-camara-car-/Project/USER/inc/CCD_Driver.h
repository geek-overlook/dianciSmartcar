#ifndef _CCD_DRIVER_H_
#define _CCD_DRIVER_H_

#include "headfile.h"
#include "common.h"
#include "zf_uart.h"
#include "board.h"
#include "buzzer.h"

//�����޷�
#define LimitL(L) (L=((L<15)?15:L))
#define LimitH(H) (H=((H>112)?112:H))

extern uint8 mid_point;
extern uint8 left_point;
extern uint8 right_point;
extern uint16 ccd_data_01[128];
extern uint16 ccd_data_02[128];
extern uint8 count_ones_ccd1;
extern uint8 count_ones_ccd2;
extern uint8 last_count_ones_ccd1;
extern uint8 last_count_ones_ccd2;
extern int8 offset1;//ccd1ƫ��ֵ
extern int8 offset2;//ccd1ƫ��ֵ
extern int8 last_offset;
extern int8 offset_inertia;//编码器�?算的�?��
extern uint8 ccd_select;
extern uint8  ccd1_l_loss;
extern uint8  ccd1_r_loss;
extern uint8  ccd2_l_loss;
extern uint8  ccd2_r_loss;
extern uint16 ccd1_w_count;
extern uint16 ccd2_w_count;
extern uint8 zebra_count_stop;
extern uint16 ccd_value1;
extern uint16 ccd_value2;
extern uint8 Now_left_point2;
extern uint8 Now_right_point2;
extern float aim_angle_filter;
extern uint8 tof_enable;
extern uint8 last_mid_point2;
//ccd1���ұߵ�
extern uint8 left_point1;
extern uint8 right_point1;
//ccd2���ұߵ�
extern uint8 left_point2;
extern uint8 right_point2;
//��һ��ccd1���ұߵ�
extern uint8 last_left_point1; 
extern uint8 last_right_point1;
extern uint8 last_left_point2; 
extern uint8 last_right_point2;

extern uint8 circle_sideline_count;
extern uint8 circle_distance ;
extern uint8 circle_count;  //����״̬�ƴ�
extern uint8 circle_left_in_flag;//�뻷
extern uint8 circle_right_in_flag;
extern uint8 circle_left_out_flag;//����
extern uint8 circle_right_out_flag;
extern uint8 circle_left_out;//����
extern uint8 ccd1_circle_left_out;//����
extern uint8 l_circle_max; //������������
extern int8 last_offset2;
extern uint16 speed_stright;
extern uint16 speed_turn ;
extern int16  limit_motor_pwm;
extern uint32 turn_kp;
extern uint32 turn_kd;
extern uint32 inertia_kp;
extern uint32 inertia_kd;
extern uint16 speed_cross ;
extern uint16 element_ccd_value; 


/////////////////////////////////Ԫ�ؼ���־λ//////////////////////////////
extern uint8 circle_left_check;
extern uint8 circle_left_big_check;
extern uint8 circle_right_check;
extern uint8 cross_check;
extern uint8 cross_circle_cleck;
extern uint8 ramp_check;
extern uint8 zebra_flag;

enum flag_circle_e {
    CIRCLE_NONE = 0,  //�ǻ���
    CIRCLE_LEFT_BEGIN,//ʶ�𵽻�����δ����Բ��ˮƽλ��
    CIRCLE_RIGHT_BEGIN,
    CIRCLE_LEFT_IN,
    CIRCLE_RIGHT_IN,  //����Բ��ˮƽλ�ã����뻷��
    CIRCLE_LEFT_RUNNING,
    CIRCLE_RIGHT_RUNNING,  //�ڻ�������
    CIRCLE_LEFT_OUT,
    CIRCLE_RIGHT_OUT,  //���»ص�BEGIN��λ�ã�����
    CIRCLE_LEFT_END,
	CIRCLE_RIGHT_END,
};

enum flag_cross_circle_e {
    CROSS_CIRCLE_NONE = 0,
    CROSS_CIRCLE_LEFT_RUNNING,
    CROSS_CIRCLE_RIGHT_RUNNING,
    CROSS_CIRCLE_OUT,
};

enum flag_ramp_e {
    RAMP_NONE = 0,
    RAMP_BIGIN,
    RAMP_IN_UP,
    RAMP_IN_MID,
    RAMP_IN_DOWN,
};

enum flag_cross_e {
    CROSS_NONE = 0,
    CROSS_MID,
    CROSS_RUNNING,
};

extern enum flag_circle_e flag_circle;
extern enum flag_cross_circle_e flag_cross_circle;
extern enum flag_ramp_e flag_ramp;
extern enum flag_cross_e flag_cross;

uint16 ccd_threshold(uint16 ccd_data[]);
uint16 myabs(int16 number);
void get_mid_point1(void); 
void get_mid_point2(void); 
void ccd_two_value(uint16 value1, uint16 value2);
void  get_image_value(void);
void get_offset(void);
uint16 zebra_stop(uint16 arr[], uint16 n);
void Bin_CCD_Filter();
#endif