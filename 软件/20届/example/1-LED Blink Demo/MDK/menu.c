#include "headfile.h"
#include "control.h"
#include "Menu.h"
#include "PID.h"
#include "MENU_DATA.h"





//按键
//宽度120，高度140

uint8 show_flag,last_show;
uint8 key_last,key_down;
uint8 key_val;
//有刷

//int32 encoder_Sum1=0,encoder_Sum2=0;
float Y_distance,Y_distance2;//距离

//运行时间
int32 time1,time2,time;

#define up     1
#define down   2
#define left   3
#define right  4
#define mid_b    5
#define bluetwo   6 //启动按键
#define blueone   7

int num_up_dowm_one;//一级菜单上下移动
int num_up_dowm_two;//二级菜单上下移动
int num_up_dowm_three;//三级菜单上下移动
int num_right_left_flag=2;//加减数值标志位，1可加，0可减法，2锁死
int num_choice_confirm[5]={0,0,0,0,0};//按下确认锁死标志位，数组分为不同菜单的按下确认，1一级菜单锁死，2二级菜单锁死，3三级菜单锁死，4四级
uint8 clear_menu_flag=0;//换菜单的时候清空前面的菜单
//uint8 first_menu_flag=1;//换二级菜单的时候，只让一级菜单显示一次
uint8 menu_show_flag[15]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

uint8 first_confirm_flag=1;//一级菜单的上下锁死标志位，0锁死，1可加减
uint8 mid_change=0;//按下确认键的菜单改变标志，用于数组的选择，0-一级菜单，1-二级，2-三级
uint8 second_confirm_flag=0;//二级菜单锁死标志位0锁死，1可加减
uint8 thrid_confirm_flag=0;//三级菜单锁死标志位0锁死，1可加减

uint8 symbol_flag_show[3]={0};
uint8 symbol_flag_show1=1;//一>显示
uint8 symbol_flag_show2=1;//二>显示
uint8 symbol_flag_show3=1;//三>显示
uint8 symbol_flag_show4=1;//四>显示
uint8 symbol_flag_show5=1;//四>显示

uint8 first_y=1;
uint8 first_x=3;

int menu_limt[3] = {6,0,0};

void show_meun(int choice,int meun_num);
void show_data(void);
uint8 last_show_flag;





 
#define MainMenu_num 6
int car_start=1;
int num_right_left=1;//加减数值数
int layer[2]= {1,5};
int layer_last[2]= {0,0};


/*y范围80-150，x范围10-120*/
void lcd_choice_function()//显示参数名以及数值
{
    /*显示级数，成员数量*/
    show_meun(layer[0],6);
    show_data();
}

// *****************临时变量********************//
int cross_dir[10] = {0};  //0直走，1左转，2右转
//int Rmid_thr=IMGW-1-35,Lmid_thr=30;
float end_angle,start_angle,none_distance;
/*断路*/
float end_angle1,end_angle2,end_angle3;
float none_distance1,none_distance2,none_distance3;
/*路障*/
int Barrier_limit1=30,Barrier_limit2=-40;
float gain1=160,gain2=80;
int Zebra_dir,end_num=60;
float Zebra_speed = 2000;
float acc=10;
int none_test;
float NYaw,NDis,NSpeed,Nerrlim;
int _180du;
float k_e1,k_e2,k_e3,k_e4,k_e5;
int firing_flag = 0,image_flag=0,wifi_flag=0;

//临时变量区
int start_speed=0;
int front_dot_s;
int k_lost;
int speed_chose;


/*摄像头参数*/
int Mt9v_exp=150,Mt9v_gain=20;//曝光和增益
int speed_chose = 3;
float speed_val[8]={
        2000,2300,2500,2700,2800,2900,3000,3100
};

/*拿个拨码开关来决定车的电机
 * 还有看圆环的，看十字，看路障的，快速切换用*/
/*主菜单*/
const char* MainMenu_lable[10]={
   "Img","Speed","PID","Nstart","Otsu","Start"
};

const char* _2Menu_lable[10][10]={
        { "show_image","Wifi","front","k_lost","S_mode","return"}, /*有下一级*/
        { "return"  },
        { "G_PID", "W_PID","T_PID", "S_PID","Servo_P","return"},                  /*有下一级*/
        { "Cross", "Circle","Ramp","Barrier","center_s","return" },
        {"Lthres","Hthres","return" }//下限阈值
};

const char* _3Menu_lable[20][10]={
        /*********************元素********************/
        { "Gyroy.kp","Gyroy.ki","return"},                               /*角速度*/
        { "WPitch.kp", "WPitch.kd","return"},              /*角度*/
        { "ramp", "common","circle","turn","RT","Yaw.kp","Turn.kp","return"},                /*转向*/
        { "Speed.kp", "Speed.kd","return"},              /*曲线减速*/
        {"Servo_P","return"},            /*车库*/
        { "front_s","return"}                                /*加速度*/

        /*********************PID********************/
};




//整个菜单的总体逻辑是按键通过key_process()函数lcd_choice_function函数显示参数名以及数值,如下

    // while (TRUE)
    // {
    //     key_process();//按键更改layer[0],layer[1],通过adjust函数修改参数
    //     if(firing_flag==0)
    //     {
    //         lcd_choice_function();//显示参数名以及数值
    //     }
    // }





//lcd_choice_function中有show_menu和show_data两个函数，show_menu在屏幕左侧显示参数名，show_data在右侧显示其数值的（例如show_menu中在左侧显示X,show_data在右侧显示1，代表X=1）
//key_process中，当key_down的值为"up"或"down"时，控制layer[1]的加减来控制">"的光标显示在第几行，当key_down的值为"left"和"right"时，调用adjust这个函数
//来控制参数的加减并将它存储于flash中,当key_down的值为mid_b时，会通过光标的位置来选择退回上一级菜单还是进入下一级菜单




//layer[0]的值表示现在为几级菜单，等于一就是一级菜单;layer[1]表示当前光标在第几行
//显示光标的逻辑：打个比方，光标初始在第一行，要让它往下移一格，就要先消除第一行的光标再显示第二行的光标，故显示逻辑为先用layer_last[1]赋值layer[1]，然后通过"up"或"down"
//控制layer[1]加减，之后再在layer_last[1]行显示" "消除光标，在layer[1]行显示">"显示光标，以此控制光标上下移，如下（246行）：

// case up:
//         layer_last[1]=layer[1];//layer_last用于清除">"
//         layer[1]--;    





//先将上面的一二三级菜单数组分别简称为_M[],_2M[][],_3M[][]
//当key_down的值为mid_b时怎么控制进出菜单：先打个比方，菜单初始时从上到下显示{"Img","Speed","PID","Nstart","Otsu","Start"}这几个单词，光标显示在"Img"这行，把光标移到"PID"
//这行，就是第三行然后按下按键将key_down置mid_b，显示屏就会显示第二级菜单第三项{ "G_PID", "W_PID","T_PID", "S_PID","Servo_P","return"}这里，然后光标移至"return"这里，再
//按一下就会回到第一级菜单。首先光标移到第三行时layer[0l==1,ayer[1]==3，按一下按键layer[0]++,即级数加一，然后更改显示内容是show_menu的任务，show_menu中先定义一个nest_layer
//将其赋值layer[1],也就是3，然后清除后用循环显示_2M[nest_layer][i]就实现了，如下（298行）：

// if(choice>last_choice) {  lcd_clear(WHITE); last_choice = choice; nest_layer = layer[1]; layer[1] = 0;}
// if(choice<last_choice) {  lcd_clear(WHITE); last_choice = choice; nest_layer = last_layer; layer[1] = 0;}//这里choice在调用时被赋值layer[0],last_layer为无效变量

//  switch(choice)
//     {
//         case 1: /*主菜单*/

//            /*显示,并清除>*/
//            lcd_showstr(0,first_y+layer_last[1]*10," ");  //清除之前行的">"
//            lcd_showstr(0,first_y+layer[1]*10, ">");  //显示当前行">"
//            nest_layer=0;
//            for(i=0;i<10;i++)
//            {
//              if(MainMenu_lable[i]==NULL) break;
//              lcd_showstr(first_x,first_y+i*10,   MainMenu_lable[i]);//显示各项参数名
//              return_labelx=99;
//              return_labely=i;//循环结束i的值即为最后一项，用于return的返回逻辑
//            }





//adjust（int type）就很好理解了，通过上面的例子就可以理解layer[0]表示第几级菜单，nest_layer表示上一次光标在第几行，通过这两个就可以确定你显示哪个数组里，然后layer[0]表示你
// 当前级光标显示在哪，通过这三个值就可以确定修改哪个参数了




//最后说一下int return_labelx=99,return_labely=99;这两个变量，这两个变量就只有99,2,3这三个值，99代表第一级菜单时，它们用来实现当光标处在return这行时能实现退回到上一级
// 菜单的操作，如下代码（267行）：
//case mid_b:
//            if(return_labelx == layer[0] && return_labely == layer[1]) {
                 // 用户选择了返回操作，降低一个菜单层级
//                 layer[0]--;
//                 if(layer[0] < 1) {
//                     layer[0] = 1; // 确保不会低于主菜单层级
//                }
//这逻辑是当return_labelx为当前级数且return_labely为当前光标行数时按下去退回上一级，return_labelx赋值就行了，主要是如何确定return_labely一定为最后一行(上面数组定义里
// “return”都是最后一个)，代码如下（324行）：

// for(i=0;i<10;i++)
//              {
//                  if(_2Menu_lable[nest_layer][i]==NULL) break;
//                  lcd_showstr(first_x,80+i*10,   _2Menu_lable[nest_layer][i]);//显示数组内的名称
//                  return_labelx=2;
//                  return_labely=i;//循环结束i的值即为最后一项，用于return的返回逻辑
//              }




void adjust_process(int type);
int last_choice=1;
int nest_layer,last_layer;//nest_layer储存上一级的项数，last为下一级(目前为无效参数)
int return_labelx=99,return_labely=99;


void key_process(void)//按键更改layer[0],layer[1],通过adjust函数修改参数
{
    key_val=key_scan(1);
    key_down=key_val&(key_val^key_last);  //消抖
    key_last=key_val;
    switch(key_down)
  {
      case up:
          layer_last[1]=layer[1];//layer_last用于清除">"
          layer[1]--;    
          if(layer[1]<0) layer[1] = return_labely;//限幅和清零
          break;

      case down:

          layer_last[1]=layer[1];//layer_last用于清除">"
          layer[1]++;
          if(layer[1]>return_labely) layer[1] = 0;
          break;
    /*左减右加*/
      case right:
          adjust_process(1);
              break;

      case left:
          adjust_process(0);
              break;

        case mid_b:
            if(return_labelx == layer[0] && return_labely == layer[1]) {
                // 用户选择了返回操作，降低一个菜单层级
                layer[0]--;
                if(layer[0] < 1) {
                    layer[0] = 1; // 确保不会低于主菜单层级
                }
            } else {
                // 用户选择进入下一层级或在同一层级内移动
                layer_last[0] = layer[0]; // 记录上一个层级，可能未来有用

                if(layer[0] < 3) {
                    layer[0]++; // 仅当不在最深层级时，才允许进入更深层级
                }

                last_layer = nest_layer; 
            }

               break;

        default: break;

    }
}

//nest_layer用于选择二级菜单该进入什么界面
//

void show_meun(int choice,int meun_num)//choice赋值为layer[0]
{
    int i=0;
    /*更新界面*/
    if(choice>last_choice) {  lcd_clear(WHITE); last_choice = choice; nest_layer = layer[1]; layer[1] = 0;}//choice为菜单级,nest_layer赋值为layer[1],即上一级菜单的第x项
    if(choice<last_choice) {  lcd_clear(WHITE); last_choice = choice; nest_layer = last_layer; layer[1] = 0;}//nest_layer为上一级项数，layer[0]为当前级项数，递进时清零
    switch(choice)
    {
        case 1: /*主菜单*/

           /*显示,并清除>*/
           lcd_showstr(0,first_y+layer_last[1]*10," ");  //清除之前行的">"
           lcd_showstr(0,first_y+layer[1]*10, ">");  //显示当前行">"
           nest_layer=0;
           for(i=0;i<10;i++)
           {
             if(MainMenu_lable[i]==NULL) break;
             lcd_showstr(first_x,first_y+i*10,   MainMenu_lable[i]);//显示各项参数名
             return_labelx=99;
             return_labely=i;//循环结束i的值即为最后一项，用于return的返回逻辑
           }

        break;
        case 2: /*2级菜单*/
        {
         /*显示,并清除>*/
         if(layer[0]==2&&nest_layer==5)//第一个数组里的“Start”
         {
             lcd_showstr(0,80+layer_last[1]*10," ");  //清除之前行的">"
             lcd_showstr(0,80+layer[1]*10, ">");  //显示当前行">"
             for(i=0;i<10;i++)
             {
                 if(_2Menu_lable[nest_layer][i]==NULL) break;
                 lcd_showstr(first_x,80+i*10,   _2Menu_lable[nest_layer][i]);//显示数组内的名称
                 return_labelx=2;
                 return_labely=i;//循环结束i的值即为最后一项，用于return的返回逻辑

             }
         }
         else
         {
             lcd_showstr(0,first_y+layer_last[1]*10," ");  //清除之前行的">"
             lcd_showstr(0,first_y+layer[1]*10, ">");  //显示当前行">"
             for(i=0;i<10;i++)
            {
                if(_2Menu_lable[nest_layer][i]==NULL) break;
                lcd_showstr(first_x,first_y+i*10,   _2Menu_lable[nest_layer][i]);//显示数组内的名称
                return_labelx=2;
                return_labely=i;//循环结束i的值即为最后一项，用于return的返回逻辑

            }
         }
        }
        break;
        case 3: /*3级菜单*/
        {
            /*显示,并清除>*/
            lcd_showstr(0,first_y+layer_last[1]*10," ");  //清除之前行的">"
            lcd_showstr(0,first_y+layer[1]*10, ">");  //显示当前行">"
            for(i=0;i<10;i++)
            {
              if(_3Menu_lable[nest_layer][i]==NULL) break;
              lcd_showstr(first_x,first_y+i*10,   _3Menu_lable[nest_layer][i]);//显示数组内的名称
              return_labelx=3;
              return_labely=i;//循环结束i的值即为最后一项，用于return的返回逻辑
            }
        }
        break;

    }

}

int p_x=60;
int no_barrier;
int center_enter;



void show_data(void)
{
    if(layer[0]==1)
        lcd_showint16(first_x+p_x,first_y+10, start_speed );//speed_val[speed_chose]
    else if(layer[0]==2)
    {
        switch(nest_layer)
        {
            case 0:/*图像*/
                lcd_showint16(first_x+p_x+20,first_y,image_flag );
                lcd_showint16(first_x+p_x+20,first_y+10,wifi_flag );
                lcd_showint16(first_x+p_x+20,first_y+20,front_dot_s );
                lcd_showfloat(first_x+p_x+10,first_y+30,k_lost ,3,3);
                lcd_showint16(first_x+p_x+20,first_y+40,speed_chose);

                break;


//            case 3: /*舵机*/
//                lcd_showfloat(first_x+p_x+20,first_y,  Servo_PID.kp,3,2);
//                break;

            case 5: /*大津法*/
//                tft180_show_gray_image(0, 0, img_filter[0], MT9V03X_W, MT9V03X_H, MT9V03X_W / 2, MT9V03X_H, 0);
//                lcd_showint16(first_x+p_x,80,      lower_thres ,3);
//                lcd_showint16(first_x+p_x,80+10,      higth_thres ,3);
                lcd_clear(WHITE);
                firing_flag = 1;
                break;
        }
    }
    else
    {
        switch(nest_layer)
         {
            case 0:/*角速度*/
                lcd_showfloat(first_x+p_x,first_y, Gyroy_PID.kp,3,2);
                lcd_showfloat(first_x+p_x,first_y+10, Gyroy_PID.ki,3,2);
                break;

            case 1:/*角度*/
                lcd_showfloat(first_x+p_x,first_y,WPitch_PID.kp,3,2);
                lcd_showfloat(first_x+p_x,first_y+10,WPitch_PID.kd,3,2);
//                lcd_showint16(first_x+p_x,first_y+20,big_circle,3);
                break;

            case 2:/*转向环*/
                lcd_showfloat(first_x+p_x,first_y,     k_e1,3,2);
                lcd_showfloat(first_x+p_x,first_y+10,  k_e2,3,2);
                lcd_showfloat(first_x+p_x,first_y+20,  k_e3,3,2);
                lcd_showfloat(first_x+p_x,first_y+30,  k_e4,3,2);
                lcd_showfloat(first_x+p_x,first_y+40,  k_e5,3,2);
                lcd_showfloat(first_x+p_x,first_y+50,  YGyrox_PID.kp,3,2);
                lcd_showfloat(first_x+p_x,first_y+60,  Turn_PID.kp,3,3);
                break;

            case 3:/*曲线减速*/
                lcd_showfloat(first_x+p_x,first_y, Speed_PID.kp,3,2);
                lcd_showfloat(first_x+p_x,first_y+10,  Speed_PID.kd,3,2);
                break;

            case 4:/*舵机*/
                lcd_showfloat(first_x+p_x,first_y,  Servo_PID.kp,3,2);//Servo_PID.kp
                break;
//            case 5:/*加速度*/
//                lcd_showint16(first_x+p_x,first_y,  front_dot_s,3);
//                lcd_showfloat(first_x+p_x,first_y+10,  Target_Comspeed_max,4,1);
//                break;
         }
    }

}

uint32 *ptemp=0;
void adjust_process(int type)
{
    if(type)
    {
        if(layer[0]==1&&layer[1]==1)
        {
            start_speed+=100;
        }

        else if(layer[0]==2)
        {
            switch(nest_layer)
            {
                case 0:/*图像开关*/
                    if(layer[1]==0)    image_flag+=1;
                    if(layer[1]==1)		 wifi_flag+=1;   
								
                    if(layer[1]==2)    front_dot_s+=1;
                    if(layer[1]==3)    k_lost=0.001;
                    if(layer[1]==4)    //Speed_Chose(1);
                    image_flag%=2;
                    wifi_flag%=2;
                    break;

            }
        }
        else if(layer[0]==3)
        {
            switch(nest_layer)
            {
                case 0:/*角速度*/
                    if(layer[1]==0)          Gyroy_PID.kp+=0.02;
                    else if(layer[1]==1)     Gyroy_PID.ki+=0.01;
                    break;

                case 1:/*角度环*/
                    if(layer[1]==0)          WPitch_PID.kp+=0.01;
                    else if(layer[1]==1)     WPitch_PID.kd+=0.01;
                    break;

                case 2:/*转向环*/
                    if(layer[1]==0)          k_e1+=0.01;
                    else if(layer[1]==1)     k_e2+=0.01;
                    else if(layer[1]==2)     k_e3+=0.01;
                    else if(layer[1]==3)     k_e4+=0.01;
                    else if(layer[1]==4)     k_e5+=0.01;
                    else if(layer[1]==5)     YGyrox_PID.kp+=0.01;
                    else if(layer[1]==6)     Turn_PID.kp+=0.001;
                    break;

                case 3:/*减速参数*/
                    if(layer[1]==0)          Speed_PID.kp+=0.01;
                    else if(layer[1]==1)     Speed_PID.kd+=0.01;
                    break;

                case 4:/*舵机*/
                    if(layer[1]==0)          Servo_PID.kp+=0.01;

                    break;
    //            case 5:/*加速度  */
    //                if(layer[1]==0)          ;
    //                else if(layer[1]==1)     Target_Comspeed_max+=50;
    //                break;
            }
        }   
    }
/****************************------------  自减  -------------*********************************/
    else
    {
        if(layer[0]==1&&layer[1]==1)
        {
            start_speed-=100;
            if(speed_chose<0) speed_chose=0;
        }
//        else if(layer[0]==1&&layer[1]==6)
//        {
//
//        }
        else if(layer[0]==2)
        {
            switch(nest_layer)
            {
                case 0:
                    if(layer[1]==0)    image_flag-=1;
                    if(layer[1]==1)    wifi_flag-=1;
                    if(layer[1]==2)    front_dot_s-=1;
                    if(layer[1]==3)    k_lost-=0.001;
                    if(layer[1]==4)    //Speed_Chose(-1);
                    image_flag%=2;
                    wifi_flag%=2;
                    break;

            }
        }
        else if(layer[0]==3)
        {
            switch(nest_layer)
             {
                case 0:/*角速度*/
                    if(layer[1]==0)          Gyroy_PID.kp-=0.02;
                    else if(layer[1]==1)     Gyroy_PID.ki-=0.01;
                    break;

                case 1:/*角度*/
                    if(layer[1]==0)          WPitch_PID.kp-=0.01;
                    else if(layer[1]==1)     WPitch_PID.kd-=0.01;
                    break;

                case 2:/*转向*/
                    if(layer[1]==0)          k_e1-=0.01;
                    else if(layer[1]==1)     k_e2-=0.01;
                    else if(layer[1]==2)     k_e3-=0.01;
                    else if(layer[1]==3)     k_e4-=0.01;
                    else if(layer[1]==4)     k_e5-=0.01;
                    else if(layer[1]==5)     YGyrox_PID.kp-=0.01;
                    else if(layer[1]==6)     Turn_PID.kp-=0.001;
                    break;

                case 3:/*曲线减速*/
                    if(layer[1]==0)          Speed_PID.kp-=0.01;
                    else if(layer[1]==1)     Speed_PID.kd-=0.01;
                    break;

                case 4:/*舵机*/
                    if(layer[1]==0)          Servo_PID.kp-=0.01;
                    break;

//                case 5:/*加速度*/
//                    if(layer[1]==0)          front_dot_s-=1;
//                    else if(layer[1]==1)     Target_Comspeed_max-=50;
//                    break;
             }
        }
    }

    /*后面改为1s刷新一次*/
        write_flash();//按了加减才能写入
//        if(speed_mode==1)
//        {
//            if(flash_check(FLASH_SECTION_INDEX, SPEED_SMOOTH))
//               flash_erase_page(FLASH_SECTION_INDEX, SPEED_SMOOTH);
//            flash_write_page_from_buffer(FLASH_SECTION_INDEX, SPEED_SMOOTH);
//        }
//        else if(speed_mode==2)
//        {
//            if(flash_check(FLASH_SECTION_INDEX, SPEED_FAST))
//               flash_erase_page(FLASH_SECTION_INDEX, SPEED_FAST);
//            flash_write_page_from_buffer(FLASH_SECTION_INDEX, SPEED_FAST);
//        }
//        else if(speed_mode==3)
//        {
//            if(flash_check(FLASH_SECTION_INDEX, SPEED_DARTING))
//               flash_erase_page(FLASH_SECTION_INDEX, SPEED_DARTING);
//            flash_write_page_from_buffer(FLASH_SECTION_INDEX, SPEED_DARTING);
//        }
//        else if(speed_mode==4)
//        {
//            if(flash_check(FLASH_SECTION_INDEX, SPEED_DRAG_RACING))
//               flash_erase_page(FLASH_SECTION_INDEX, SPEED_DRAG_RACING);
//            flash_write_page_from_buffer(FLASH_SECTION_INDEX, SPEED_DRAG_RACING);
//        }
}













