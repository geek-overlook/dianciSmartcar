#ifndef _MOTOR_H
#define _MOTOR_H
#include "headfile.h"

extern uint16 r_encoder;
extern uint16 l_encoder;
extern float r_speed,l_speed;


extern float calculate_speed(uint16 encoder_value, int direction);
extern float forwardfeed_L(float in);
extern float forwardfeed_R(float inc_in);			

#endif 

