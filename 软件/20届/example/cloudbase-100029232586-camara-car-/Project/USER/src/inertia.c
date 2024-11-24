#include "inertia.h"

float X_acc = 0;
float Y_acc = 0;
float Z_acc = 0;
float X_gyro = 0;
float Y_gyro = 0;
float Z_gyro = 0;
float yaw = 0,target_yaw = 0,turn_pro_yaw = 0;
uint8 turn_pro_flag = 99;//0左1右
uint8 flag_turn_pro = 0;
float offset_r = 0,offset_r_x=0,offset_r_d=0;
//固定角度与标志位，在需要固定打角的地方置1,不用时及时置零
int8 offset_keep = 0;
int8 offset_keep_flag = 0;


//计算惯导偏差值，只需在定时器调用一次，多个元素不用多次调用
//用法：1.在元素处理过程设定target_yaw
//     2.在巡线方式设置在该元素用offset_inertia作为偏差值
void get_inertia_offset(void){
    int16 temp_offset=0;
    temp_offset = yaw-target_yaw;
    if(temp_offset > 180){
        temp_offset-=360; 
    }else if(temp_offset < -180){
        temp_offset+=360;
    }

    if(temp_offset>100){
        temp_offset = 100;
    }else if (temp_offset<-100){
        temp_offset = -100;
    }
    offset_inertia = -(int8)temp_offset;
}

//计算当前yaw与输入yaw偏差值，1左偏差(左环岛），0右偏差
//输出0~360
float yaw_error(float yaw_angle,uint8 mode){
    float error=0;
    if(mode){
        error = yaw - yaw_angle;
    }else{
        error = yaw_angle - yaw;
    }


    if(error > 360){
        error -= 360; 
    }else if(error < 0){
        error += 360;
    }
    return error;
}

//0：左环
//1：右环
void get_target_yaw_cross_cricle(int8 mode){
    if(mode){
        target_yaw = cross_circle_yaw;
        target_yaw -= 270;
    }else{
        target_yaw = cross_circle_yaw;
        target_yaw += 270;
    }

    if(target_yaw > 360){
        target_yaw -= 360; 
    }else if(target_yaw < 0){
        target_yaw += 360;
    }
}

void get_target_yaw(int8 angle){
//    if(angle >= -5 && angle <= 5){
//        target_yaw = yaw;
//        target_yaw = angle;
//    }
//    else if (angle >= -6 && angle <= -5){
//        target_yaw = yaw;
//        target_yaw += angle;
//        target_yaw -= 3;
//    }
//    else if (angle >=- 11 && angle <= -7){
//        target_yaw = yaw;
//        target_yaw += angle;
//        target_yaw -= 4;
//    }
//    else if (angle >=-15 && angle <= -12){
//        target_yaw = yaw;
//        target_yaw += angle;
//        target_yaw -= 5;
//    }
//    else if (angle >=-20 && angle <= -15){
//        target_yaw = yaw;
//        target_yaw += angle;
//        target_yaw -= 5;
//    }
//    else if (angle >= 6 && angle <= 20){
//        target_yaw = yaw;
//        target_yaw += angle;
//    }else{
        target_yaw = yaw;
//    }

    
//    if(target_yaw < 0)target_yaw += 360;
//    else if(target_yaw > 360)target_yaw = 0;

}



