#include "headfile.h"
#include "control.h"
#include "Menu.h"
#include "PID.h"
#include "MENU_DATA.h"





//����
//���120���߶�140

uint8 show_flag,last_show;
uint8 key_last,key_down;
uint8 key_val;
//��ˢ

//int32 encoder_Sum1=0,encoder_Sum2=0;
float Y_distance,Y_distance2;//����

//����ʱ��
int32 time1,time2,time;

#define up     1
#define down   2
#define left   3
#define right  4
#define mid_b    5
#define bluetwo   6 //��������
#define blueone   7

int num_up_dowm_one;//һ���˵������ƶ�
int num_up_dowm_two;//�����˵������ƶ�
int num_up_dowm_three;//�����˵������ƶ�
int num_right_left_flag=2;//�Ӽ���ֵ��־λ��1�ɼӣ�0�ɼ�����2����
int num_choice_confirm[5]={0,0,0,0,0};//����ȷ��������־λ�������Ϊ��ͬ�˵��İ���ȷ�ϣ�1һ���˵�������2�����˵�������3�����˵�������4�ļ�
uint8 clear_menu_flag=0;//���˵���ʱ�����ǰ��Ĳ˵�
//uint8 first_menu_flag=1;//�������˵���ʱ��ֻ��һ���˵���ʾһ��
uint8 menu_show_flag[15]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

uint8 first_confirm_flag=1;//һ���˵�������������־λ��0������1�ɼӼ�
uint8 mid_change=0;//����ȷ�ϼ��Ĳ˵��ı��־�����������ѡ��0-һ���˵���1-������2-����
uint8 second_confirm_flag=0;//�����˵�������־λ0������1�ɼӼ�
uint8 thrid_confirm_flag=0;//�����˵�������־λ0������1�ɼӼ�

uint8 symbol_flag_show[3]={0};
uint8 symbol_flag_show1=1;//һ>��ʾ
uint8 symbol_flag_show2=1;//��>��ʾ
uint8 symbol_flag_show3=1;//��>��ʾ
uint8 symbol_flag_show4=1;//��>��ʾ
uint8 symbol_flag_show5=1;//��>��ʾ

uint8 first_y=1;
uint8 first_x=3;

int menu_limt[3] = {6,0,0};

void show_meun(int choice,int meun_num);
void show_data(void);
uint8 last_show_flag;





 
#define MainMenu_num 6
int car_start=1;
int num_right_left=1;//�Ӽ���ֵ��
int layer[2]= {1,5};
int layer_last[2]= {0,0};


/*y��Χ80-150��x��Χ10-120*/
void lcd_choice_function()//��ʾ�������Լ���ֵ
{
    /*��ʾ��������Ա����*/
    show_meun(layer[0],6);
    show_data();
}

// *****************��ʱ����********************//
int cross_dir[10] = {0};  //0ֱ�ߣ�1��ת��2��ת
//int Rmid_thr=IMGW-1-35,Lmid_thr=30;
float end_angle,start_angle,none_distance;
/*��·*/
float end_angle1,end_angle2,end_angle3;
float none_distance1,none_distance2,none_distance3;
/*·��*/
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

//��ʱ������
int start_speed=0;
int front_dot_s;
int k_lost;
int speed_chose;


/*����ͷ����*/
int Mt9v_exp=150,Mt9v_gain=20;//�ع������
int speed_chose = 3;
float speed_val[8]={
        2000,2300,2500,2700,2800,2900,3000,3100
};

/*�ø����뿪�����������ĵ��
 * ���п�Բ���ģ���ʮ�֣���·�ϵģ������л���*/
/*���˵�*/
const char* MainMenu_lable[10]={
   "Img","Speed","PID","Nstart","Otsu","Start"
};

const char* _2Menu_lable[10][10]={
        { "show_image","Wifi","front","k_lost","S_mode","return"}, /*����һ��*/
        { "return"  },
        { "G_PID", "W_PID","T_PID", "S_PID","Servo_P","return"},                  /*����һ��*/
        { "Cross", "Circle","Ramp","Barrier","center_s","return" },
        {"Lthres","Hthres","return" }//������ֵ
};

const char* _3Menu_lable[20][10]={
        /*********************Ԫ��********************/
        { "Gyroy.kp","Gyroy.ki","return"},                               /*���ٶ�*/
        { "WPitch.kp", "WPitch.kd","return"},              /*�Ƕ�*/
        { "ramp", "common","circle","turn","RT","Yaw.kp","Turn.kp","return"},                /*ת��*/
        { "Speed.kp", "Speed.kd","return"},              /*���߼���*/
        {"Servo_P","return"},            /*����*/
        { "front_s","return"}                                /*���ٶ�*/

        /*********************PID********************/
};




//�����˵��������߼��ǰ���ͨ��key_process()����lcd_choice_function������ʾ�������Լ���ֵ,����

    // while (TRUE)
    // {
    //     key_process();//��������layer[0],layer[1],ͨ��adjust�����޸Ĳ���
    //     if(firing_flag==0)
    //     {
    //         lcd_choice_function();//��ʾ�������Լ���ֵ
    //     }
    // }





//lcd_choice_function����show_menu��show_data����������show_menu����Ļ�����ʾ��������show_data���Ҳ���ʾ����ֵ�ģ�����show_menu���������ʾX,show_data���Ҳ���ʾ1������X=1��
//key_process�У���key_down��ֵΪ"up"��"down"ʱ������layer[1]�ļӼ�������">"�Ĺ����ʾ�ڵڼ��У���key_down��ֵΪ"left"��"right"ʱ������adjust�������
//�����Ʋ����ļӼ��������洢��flash��,��key_down��ֵΪmid_bʱ����ͨ������λ����ѡ���˻���һ���˵����ǽ�����һ���˵�




//layer[0]��ֵ��ʾ����Ϊ�����˵�������һ����һ���˵�;layer[1]��ʾ��ǰ����ڵڼ���
//��ʾ�����߼�������ȷ�������ʼ�ڵ�һ�У�Ҫ����������һ�񣬾�Ҫ��������һ�еĹ������ʾ�ڶ��еĹ�꣬����ʾ�߼�Ϊ����layer_last[1]��ֵlayer[1]��Ȼ��ͨ��"up"��"down"
//����layer[1]�Ӽ���֮������layer_last[1]����ʾ" "������꣬��layer[1]����ʾ">"��ʾ��꣬�Դ˿��ƹ�������ƣ����£�246�У���

// case up:
//         layer_last[1]=layer[1];//layer_last�������">"
//         layer[1]--;    





//�Ƚ������һ�������˵�����ֱ���Ϊ_M[],_2M[][],_3M[][]
//��key_down��ֵΪmid_bʱ��ô���ƽ����˵����ȴ���ȷ����˵���ʼʱ���ϵ�����ʾ{"Img","Speed","PID","Nstart","Otsu","Start"}�⼸�����ʣ������ʾ��"Img"���У��ѹ���Ƶ�"PID"
//���У����ǵ�����Ȼ���°�����key_down��mid_b����ʾ���ͻ���ʾ�ڶ����˵�������{ "G_PID", "W_PID","T_PID", "S_PID","Servo_P","return"}���Ȼ��������"return"�����
//��һ�¾ͻ�ص���һ���˵������ȹ���Ƶ�������ʱlayer[0l==1,ayer[1]==3����һ�°���layer[0]++,��������һ��Ȼ�������ʾ������show_menu������show_menu���ȶ���һ��nest_layer
//���丳ֵlayer[1],Ҳ����3��Ȼ���������ѭ����ʾ_2M[nest_layer][i]��ʵ���ˣ����£�298�У���

// if(choice>last_choice) {  lcd_clear(WHITE); last_choice = choice; nest_layer = layer[1]; layer[1] = 0;}
// if(choice<last_choice) {  lcd_clear(WHITE); last_choice = choice; nest_layer = last_layer; layer[1] = 0;}//����choice�ڵ���ʱ����ֵlayer[0],last_layerΪ��Ч����

//  switch(choice)
//     {
//         case 1: /*���˵�*/

//            /*��ʾ,�����>*/
//            lcd_showstr(0,first_y+layer_last[1]*10," ");  //���֮ǰ�е�">"
//            lcd_showstr(0,first_y+layer[1]*10, ">");  //��ʾ��ǰ��">"
//            nest_layer=0;
//            for(i=0;i<10;i++)
//            {
//              if(MainMenu_lable[i]==NULL) break;
//              lcd_showstr(first_x,first_y+i*10,   MainMenu_lable[i]);//��ʾ���������
//              return_labelx=99;
//              return_labely=i;//ѭ������i��ֵ��Ϊ���һ�����return�ķ����߼�
//            }





//adjust��int type���ͺܺ�����ˣ�ͨ����������ӾͿ������layer[0]��ʾ�ڼ����˵���nest_layer��ʾ��һ�ι���ڵڼ��У�ͨ���������Ϳ���ȷ������ʾ�ĸ������Ȼ��layer[0]��ʾ��
// ��ǰ�������ʾ���ģ�ͨ��������ֵ�Ϳ���ȷ���޸��ĸ�������




//���˵һ��int return_labelx=99,return_labely=99;������������������������ֻ��99,2,3������ֵ��99�����һ���˵�ʱ����������ʵ�ֵ���괦��return����ʱ��ʵ���˻ص���һ��
// �˵��Ĳ��������´��루267�У���
//case mid_b:
//            if(return_labelx == layer[0] && return_labely == layer[1]) {
                 // �û�ѡ���˷��ز���������һ���˵��㼶
//                 layer[0]--;
//                 if(layer[0] < 1) {
//                     layer[0] = 1; // ȷ������������˵��㼶
//                }
//���߼��ǵ�return_labelxΪ��ǰ������return_labelyΪ��ǰ�������ʱ����ȥ�˻���һ����return_labelx��ֵ�����ˣ���Ҫ�����ȷ��return_labelyһ��Ϊ���һ��(�������鶨����
// ��return���������һ��)���������£�324�У���

// for(i=0;i<10;i++)
//              {
//                  if(_2Menu_lable[nest_layer][i]==NULL) break;
//                  lcd_showstr(first_x,80+i*10,   _2Menu_lable[nest_layer][i]);//��ʾ�����ڵ�����
//                  return_labelx=2;
//                  return_labely=i;//ѭ������i��ֵ��Ϊ���һ�����return�ķ����߼�
//              }




void adjust_process(int type);
int last_choice=1;
int nest_layer,last_layer;//nest_layer������һ����������lastΪ��һ��(ĿǰΪ��Ч����)
int return_labelx=99,return_labely=99;


void key_process(void)//��������layer[0],layer[1],ͨ��adjust�����޸Ĳ���
{
    key_val=key_scan(1);
    key_down=key_val&(key_val^key_last);  //����
    key_last=key_val;
    switch(key_down)
  {
      case up:
          layer_last[1]=layer[1];//layer_last�������">"
          layer[1]--;    
          if(layer[1]<0) layer[1] = return_labely;//�޷�������
          break;

      case down:

          layer_last[1]=layer[1];//layer_last�������">"
          layer[1]++;
          if(layer[1]>return_labely) layer[1] = 0;
          break;
    /*����Ҽ�*/
      case right:
          adjust_process(1);
              break;

      case left:
          adjust_process(0);
              break;

        case mid_b:
            if(return_labelx == layer[0] && return_labely == layer[1]) {
                // �û�ѡ���˷��ز���������һ���˵��㼶
                layer[0]--;
                if(layer[0] < 1) {
                    layer[0] = 1; // ȷ������������˵��㼶
                }
            } else {
                // �û�ѡ�������һ�㼶����ͬһ�㼶���ƶ�
                layer_last[0] = layer[0]; // ��¼��һ���㼶������δ������

                if(layer[0] < 3) {
                    layer[0]++; // ������������㼶ʱ��������������㼶
                }

                last_layer = nest_layer; 
            }

               break;

        default: break;

    }
}

//nest_layer����ѡ������˵��ý���ʲô����
//

void show_meun(int choice,int meun_num)//choice��ֵΪlayer[0]
{
    int i=0;
    /*���½���*/
    if(choice>last_choice) {  lcd_clear(WHITE); last_choice = choice; nest_layer = layer[1]; layer[1] = 0;}//choiceΪ�˵���,nest_layer��ֵΪlayer[1],����һ���˵��ĵ�x��
    if(choice<last_choice) {  lcd_clear(WHITE); last_choice = choice; nest_layer = last_layer; layer[1] = 0;}//nest_layerΪ��һ��������layer[0]Ϊ��ǰ���������ݽ�ʱ����
    switch(choice)
    {
        case 1: /*���˵�*/

           /*��ʾ,�����>*/
           lcd_showstr(0,first_y+layer_last[1]*10," ");  //���֮ǰ�е�">"
           lcd_showstr(0,first_y+layer[1]*10, ">");  //��ʾ��ǰ��">"
           nest_layer=0;
           for(i=0;i<10;i++)
           {
             if(MainMenu_lable[i]==NULL) break;
             lcd_showstr(first_x,first_y+i*10,   MainMenu_lable[i]);//��ʾ���������
             return_labelx=99;
             return_labely=i;//ѭ������i��ֵ��Ϊ���һ�����return�ķ����߼�
           }

        break;
        case 2: /*2���˵�*/
        {
         /*��ʾ,�����>*/
         if(layer[0]==2&&nest_layer==5)//��һ��������ġ�Start��
         {
             lcd_showstr(0,80+layer_last[1]*10," ");  //���֮ǰ�е�">"
             lcd_showstr(0,80+layer[1]*10, ">");  //��ʾ��ǰ��">"
             for(i=0;i<10;i++)
             {
                 if(_2Menu_lable[nest_layer][i]==NULL) break;
                 lcd_showstr(first_x,80+i*10,   _2Menu_lable[nest_layer][i]);//��ʾ�����ڵ�����
                 return_labelx=2;
                 return_labely=i;//ѭ������i��ֵ��Ϊ���һ�����return�ķ����߼�

             }
         }
         else
         {
             lcd_showstr(0,first_y+layer_last[1]*10," ");  //���֮ǰ�е�">"
             lcd_showstr(0,first_y+layer[1]*10, ">");  //��ʾ��ǰ��">"
             for(i=0;i<10;i++)
            {
                if(_2Menu_lable[nest_layer][i]==NULL) break;
                lcd_showstr(first_x,first_y+i*10,   _2Menu_lable[nest_layer][i]);//��ʾ�����ڵ�����
                return_labelx=2;
                return_labely=i;//ѭ������i��ֵ��Ϊ���һ�����return�ķ����߼�

            }
         }
        }
        break;
        case 3: /*3���˵�*/
        {
            /*��ʾ,�����>*/
            lcd_showstr(0,first_y+layer_last[1]*10," ");  //���֮ǰ�е�">"
            lcd_showstr(0,first_y+layer[1]*10, ">");  //��ʾ��ǰ��">"
            for(i=0;i<10;i++)
            {
              if(_3Menu_lable[nest_layer][i]==NULL) break;
              lcd_showstr(first_x,first_y+i*10,   _3Menu_lable[nest_layer][i]);//��ʾ�����ڵ�����
              return_labelx=3;
              return_labely=i;//ѭ������i��ֵ��Ϊ���һ�����return�ķ����߼�
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
            case 0:/*ͼ��*/
                lcd_showint16(first_x+p_x+20,first_y,image_flag );
                lcd_showint16(first_x+p_x+20,first_y+10,wifi_flag );
                lcd_showint16(first_x+p_x+20,first_y+20,front_dot_s );
                lcd_showfloat(first_x+p_x+10,first_y+30,k_lost ,3,3);
                lcd_showint16(first_x+p_x+20,first_y+40,speed_chose);

                break;


//            case 3: /*���*/
//                lcd_showfloat(first_x+p_x+20,first_y,  Servo_PID.kp,3,2);
//                break;

            case 5: /*���*/
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
            case 0:/*���ٶ�*/
                lcd_showfloat(first_x+p_x,first_y, Gyroy_PID.kp,3,2);
                lcd_showfloat(first_x+p_x,first_y+10, Gyroy_PID.ki,3,2);
                break;

            case 1:/*�Ƕ�*/
                lcd_showfloat(first_x+p_x,first_y,WPitch_PID.kp,3,2);
                lcd_showfloat(first_x+p_x,first_y+10,WPitch_PID.kd,3,2);
//                lcd_showint16(first_x+p_x,first_y+20,big_circle,3);
                break;

            case 2:/*ת��*/
                lcd_showfloat(first_x+p_x,first_y,     k_e1,3,2);
                lcd_showfloat(first_x+p_x,first_y+10,  k_e2,3,2);
                lcd_showfloat(first_x+p_x,first_y+20,  k_e3,3,2);
                lcd_showfloat(first_x+p_x,first_y+30,  k_e4,3,2);
                lcd_showfloat(first_x+p_x,first_y+40,  k_e5,3,2);
                lcd_showfloat(first_x+p_x,first_y+50,  YGyrox_PID.kp,3,2);
                lcd_showfloat(first_x+p_x,first_y+60,  Turn_PID.kp,3,3);
                break;

            case 3:/*���߼���*/
                lcd_showfloat(first_x+p_x,first_y, Speed_PID.kp,3,2);
                lcd_showfloat(first_x+p_x,first_y+10,  Speed_PID.kd,3,2);
                break;

            case 4:/*���*/
                lcd_showfloat(first_x+p_x,first_y,  Servo_PID.kp,3,2);//Servo_PID.kp
                break;
//            case 5:/*���ٶ�*/
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
                case 0:/*ͼ�񿪹�*/
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
                case 0:/*���ٶ�*/
                    if(layer[1]==0)          Gyroy_PID.kp+=0.02;
                    else if(layer[1]==1)     Gyroy_PID.ki+=0.01;
                    break;

                case 1:/*�ǶȻ�*/
                    if(layer[1]==0)          WPitch_PID.kp+=0.01;
                    else if(layer[1]==1)     WPitch_PID.kd+=0.01;
                    break;

                case 2:/*ת��*/
                    if(layer[1]==0)          k_e1+=0.01;
                    else if(layer[1]==1)     k_e2+=0.01;
                    else if(layer[1]==2)     k_e3+=0.01;
                    else if(layer[1]==3)     k_e4+=0.01;
                    else if(layer[1]==4)     k_e5+=0.01;
                    else if(layer[1]==5)     YGyrox_PID.kp+=0.01;
                    else if(layer[1]==6)     Turn_PID.kp+=0.001;
                    break;

                case 3:/*���ٲ���*/
                    if(layer[1]==0)          Speed_PID.kp+=0.01;
                    else if(layer[1]==1)     Speed_PID.kd+=0.01;
                    break;

                case 4:/*���*/
                    if(layer[1]==0)          Servo_PID.kp+=0.01;

                    break;
    //            case 5:/*���ٶ�  */
    //                if(layer[1]==0)          ;
    //                else if(layer[1]==1)     Target_Comspeed_max+=50;
    //                break;
            }
        }   
    }
/****************************------------  �Լ�  -------------*********************************/
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
                case 0:/*���ٶ�*/
                    if(layer[1]==0)          Gyroy_PID.kp-=0.02;
                    else if(layer[1]==1)     Gyroy_PID.ki-=0.01;
                    break;

                case 1:/*�Ƕ�*/
                    if(layer[1]==0)          WPitch_PID.kp-=0.01;
                    else if(layer[1]==1)     WPitch_PID.kd-=0.01;
                    break;

                case 2:/*ת��*/
                    if(layer[1]==0)          k_e1-=0.01;
                    else if(layer[1]==1)     k_e2-=0.01;
                    else if(layer[1]==2)     k_e3-=0.01;
                    else if(layer[1]==3)     k_e4-=0.01;
                    else if(layer[1]==4)     k_e5-=0.01;
                    else if(layer[1]==5)     YGyrox_PID.kp-=0.01;
                    else if(layer[1]==6)     Turn_PID.kp-=0.001;
                    break;

                case 3:/*���߼���*/
                    if(layer[1]==0)          Speed_PID.kp-=0.01;
                    else if(layer[1]==1)     Speed_PID.kd-=0.01;
                    break;

                case 4:/*���*/
                    if(layer[1]==0)          Servo_PID.kp-=0.01;
                    break;

//                case 5:/*���ٶ�*/
//                    if(layer[1]==0)          front_dot_s-=1;
//                    else if(layer[1]==1)     Target_Comspeed_max-=50;
//                    break;
             }
        }
    }

    /*�����Ϊ1sˢ��һ��*/
        write_flash();//���˼Ӽ�����д��
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













