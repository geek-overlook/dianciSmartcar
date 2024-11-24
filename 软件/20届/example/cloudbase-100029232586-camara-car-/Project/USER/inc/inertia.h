#include "headfile.h"


extern float X_acc ;
extern float Y_acc ;
extern float Z_acc ;
extern float X_gyro ;
extern float Y_gyro ;
extern float Z_gyro ;
extern float yaw ,target_yaw,turn_pro_yaw;
extern uint8 turn_pro_flag;
extern uint8 flag_turn_pro;
extern float offset_r,offset_r_x,offset_r_d;
extern int8 offset_keep ;
extern int8 offset_keep_flag;

extern void get_inertia_offset(void);
extern void get_target_yaw(int8 angle);
extern void get_target_yaw_cross_cricle(int8 mode);
extern float yaw_error(float yaw_angle,uint8 mode);