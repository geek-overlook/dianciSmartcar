#include "headfile.h"


int	motor_pwm_out = 1050;//pwm赋值给电机
int	steer_pwm_out = steet_midlle;//pwm赋值给电机,628舵机机械中值

int16 dat = 0;//编码器速度
pid_t motor_pid;
pid_t steer_pid;
pid_t inertia_pid;
unsigned char TM3_Isr_count=0;
int circle_run_point[20];
uint16 circle_begin_distence = 0;
uint16 circle_no_distence = 41000;
uint16 cross_distence = 50000;
uint16 cross_cricle_distence = 62000;
uint16 ramp_distence = 0;
uint16 tuen_pro_distence = 60000;
uint16 zebra_distence = 0;
uint8 key_mode =0;
int key_test = 0;
unsigned char process_flag = 1;
uint8 car_ready_flag = 0;//车辆状态
uint8 car_ready_gogogo = 0;//发车标志
enum count_flag_g count_crbegin_flag;
enum count_flag_g count_nocri_flag;
enum count_flag_g count_ramp_flag;
enum count_flag_g count_cross_flag;
enum count_flag_g count_cross_cricle_flag;
enum count_flag_g count_turn_pro_flag;
enum count_flag_g count_zebra_flag;
float A = 0.01;
float bas_kp = 5;
float KP = 0;
int test0 = 0;

int key_scan( int mode)
{
    static unsigned int key=1;
    if(mode)key=1;//连续扫描按键
    if(key==1&&(P16==0||P17==0||P50==0||P51==0))
    {
        delay_ms(10);//消抖
        key=0;
        if(P17==0){return 1;}
        else if(P16==0){return 2;}
        else if(P51==0){return 3;}
        else if(P50==0){return 4;}
    }
    else if(P17==1&&P16==1&&P51==1&&P50==1) //无按键按下
    {
     key=1;
    }
    return 0;
}

void addToArray(DynamicArray *dArray, uint8 value) {
    // 增加数组的大小
    uint8 *newArray = realloc(dArray->array, (dArray->size + 1) * sizeof(int));
    
    dArray->array = newArray;
    dArray->array[dArray->size] = value;
    dArray->size++;
}

//double calc_ave(DynamicArray *dArray) {
//    int sum = 0;size_t i;
//    if (dArray->size == 0) {
//        return 0.0; // 避免除以0
//    }

//    for (i = 0; i < dArray->size; i++) {
//        sum += dArray->array[i];
//    }

//    return (double)sum / dArray->size;
//}

float calc_ave(int arr[], int size) {
	int i;
	float sum = 0.0f;
	for (i = 0; i < size; i++) {
    sum += arr[i];
	}
	return sum / size;
}

//计算白块长度
int countOnes(int arr[], int size) {
  int count = 0,i;
  for (i = 0; i < size; i++) {
    if (arr[i] == 1) {
      count++;
    }
  }
  return count;
}


void KP_get(int8 ccd_offset)
{
	if(ccd_offset < 0){
		ccd_offset = -ccd_offset;
	}
	KP = (A*(float)ccd_offset*(float)ccd_offset) + (float)bas_kp;
	
	if(KP > 14)KP = 14;
	
}

float IncPID_realize(pid_t *pid, float temp_val) {
    float increment;

    pid->err = pid->target_val - temp_val; // 计算当前误差

    // 防止积分项的累积误差过大
    pid->integral += pid->err;
    if (pid->integral > 120) {
        pid->integral = 120;
    } else if (pid->integral < -70) {
        pid->integral = -70;
    }

    // 计算PID控制器的输出值增量
    increment = pid->Kp * (pid->err - pid->err_next) // 比例项
                + pid->Ki * pid->err // 积分项
                + pid->Kd * (pid->err - 2 * pid->err_next + pid->err_last); // 微分项

    pid->actual_val += increment;

    // 更新误差值
    pid->err_last = pid->err_next;
    pid->err_next = pid->err;

    // 限制输出值
    if (pid->actual_val > 500) {
        pid->actual_val = 500;
    } else if (pid->actual_val < -500) {
        pid->actual_val = -500;
    }

    return pid->actual_val;
}

// 位置 pid
float PosPID_realize(pid_t *pid, float temp_val)
{
    pid->err = (pid->target_val - temp_val);
    // 积分限幅
    pid->integral += pid->err;

    if(pid->integral > 300)
        pid->integral = 300;
    else if(pid -> integral < -70)
        pid->integral = -70;

    pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid-> err_next);
    pid->err_next = pid->err;
    return pid->actual_val;
}


void PID_param_init(pid_t *pid)
{
    memset(pid, 0, sizeof(pid));

  pid->target_val = 0.0;
    pid->actual_val = 0.0;
    pid->integral = 0.0;
    pid->err = 0.0;
    pid->err_next = 0.0;
    pid->err_last = 0.0;
    pid->Kp = 0.0;
    pid->Ki = 0.0;
    pid->Kd = 0.0;
}

// 位置式 PID 角度外环
float pid_realize_a(float actual, float set, float _p, float _d) {
    static float last_error = 0.0f;
    static float last_out_d = 0.0f;
    // static float last_actual = 0.0f;
    // static float derivative = 0.0f;

    /* 当前误差 */
    float error = set - actual;

    /* 微分先行 */
    /*
    float temp = 0.618f * _d + _p;
    float c3 = _d / temp;
    float c2 = (_d + _p) / temp;
    float c1 = 0.618f * c3;
    derivative = c1 * derivative + c2 * actual - c3 * last_actual;
    */

    /* 不完全微分 */
    float out_d = _d * 0.8f * (error - last_error) + 0.2f * last_out_d;
    // float out_d = 0.8f * derivative + 0.2f * last_out_d;

    /* 实际输出 */
    float output = _p * error + out_d;

    /* 更新参数 */
    last_error = error;
    last_out_d = out_d;
    // last_actual = actual;

    return output;
}

// 偏差滑动平均滤波
float filter(float value) 
{
    static float filter_buf[3] = {0};

	filter_buf[2] = filter_buf[1];
	filter_buf[1] = filter_buf[0];
	filter_buf[0] = value;

	return (filter_buf[2] + filter_buf[1] + filter_buf[0]) / 3.0f;
}


void key_1(){
	key_test=key_scan(0);
	if(key_test==KEY1_PRESS){motor_pid.target_val += 50;}
	if(key_test==KEY2_PRESS){motor_pid.target_val -= 50;}
	if(key_test==KEY3_PRESS){steer_pid.Kd += 0.5;}
	if(key_test==KEY4_PRESS){steer_pid.Kd -= 0.5;}
}

int max(int a, int b) {
  if (a >= b) {
    return a;
  } else {
    return b;
  }
}

//计算最大连续白点数量
int count_ones(uint16 arr[], uint16 n) {
  uint16 count = 0, max_count = 0,i;
  for ( i = 0; i < n; i++) {
    if (arr[i] == 1) {
      count++;
    } else {
      max_count = max(max_count, count);
      count = 0;
    }
  }
  max_count = max(max_count, count);  // 处理数组末尾的连续1
  return max_count;
}

//车辆开机自检，连续短鸣声自检通过
void car_check(){
  uint32 count_car_check = 0;
  P32 = 1;
  while(++count_car_check < 200000){}//延迟500ms
  count_car_check = 0;
  P32 = 0;

//第一项检查
  while(!(count_ones(ccd_data_02,128)>46&&count_ones(ccd_data_02,128)<53)){}

  while(++count_car_check < 200000){}//延迟500ms
  count_car_check = 0;
  P32 = 1;
  while(++count_car_check < 200000){}//延迟500ms
  count_car_check = 0;
  P32 = 0;

//第二项检查
  while(!(count_ones(ccd_data_01,128)>24&&count_ones(ccd_data_01,128)<32)){}

  while(++count_car_check < 200000){}//延迟500ms
  count_car_check = 0;
  P32 = 1;
  while(++count_car_check < 200000){}//延迟500ms
  count_car_check = 0;
  P32 = 0;

//第三项检查

  while(++count_car_check < 200000){}//延迟500ms
  count_car_check = 0;
  P32 = 1;
  while(++count_car_check < 200000){}//延迟500ms
  count_car_check = 0;
  P32 = 0;


  while(++count_car_check > 10000)count_car_check = 0;

  
  car_ready_flag = 1;//允许发车
}

//显示ccd二值化图像
void lcd_show_ccd(uint16 p1[],uint16 p2[])
{
	uint8 x1 = 0; //屏幕0-127
	uint8 y1 = 0; //ccd1数据
	
	uint8 x2 = 0; //屏幕0-127
	uint8 y2 = 0; //ccd1数据
	for(x1 = 0; x1 < 128; x1++)
	{
		if(p1[x1] == 1)lcd_drawpoint(128 - x1, 50, WHITE);
		else lcd_drawpoint(128 - x1, 50, RED);	
		
	}

	for(x2 = 0; x2 < 128; x2++)
	{
		if(p2[x2] == 1)lcd_drawpoint(128 - x2, 20, WHITE);
		else lcd_drawpoint(128 - x2, 20, RED);	
		
	}	
}

void car_init(void){
  lcd_init(); //初始化屏幕
	lcd_clear(BLACK); //设背景为黑色

	car_ready_flag = 0;
	
	steer_pid.Kp=7.3;
	steer_pid.Kd=17;
	steer_pid.Ki=0;
	
	PID_param_init(&motor_pid);
	motor_pid.target_val = 0;
	motor_pid.Kp = 12 ;
	motor_pid.Ki = 3;
	motor_pid.Kd = 0;

  PID_param_init(&inertia_pid);
	inertia_pid.Kp = 3;
	inertia_pid.Ki = 2.6;
	inertia_pid.Kd = 13;	

	// 无线转串口
	wireless_uart_init();
	
	
	ctimer_count_init(CTIM0_P34);	
	
	//开启中断
	pit_timer_ms(TIM_3,5);
	
	//蜂鸣器
	gpio_mode(P3_2,GPO_PP);
	P32 = 0;	
	
	gpio_mode(P2_6,GPO_PP);
	P26 = 0;	
	
//	circle_run_point = null;

  //imu660
  while(imu660ra_init()) delay_ms(500);

	pwm_init(PWMA_CH3P_P24, 17000, 0);//电机
	pwm_init(PWMB_CH3_P33, 50, steet_midlle);//舵机
	pwm_init(PWMB_CH1_P00, 50, 0);//电调
	pwm_init(PWMB_CH2_P01, 50, 0);//电调

	//EnableGlobalIRQ();
	ccd_init();    //ccd初始化
	
	//tof初始化
	while(dl1b_init()){
		delay_ms(500);
	}
    circle_left_check = 1 ;
    circle_left_big_check = 1 ;
    circle_right_check = 1 ;
    cross_check = 1 ;
    cross_circle_cleck = 1;
    ramp_check = 1 ;
}

//菜单界面0
void display_0(void){
	if (flag_ramp != RAMP_NONE){
		lcd_showstr(0, 180, "RAMP_NONE ");
	    //P32 = 0;
  }
	else if(flag_cross_circle != CROSS_CIRCLE_NONE) {
		// lcd_showstr(0, 180, "L_BIN ");
		//P32 = 0;
	}
	else if(flag_circle == CIRCLE_LEFT_BEGIN) {
		lcd_showstr(0, 180, "L_BIN ");
		//P32 = 0;
	}
	else if(flag_cross != CROSS_NONE){
		lcd_showstr(0, 180, "CROSS ");
		//P32 = 1;
	}
	else if(flag_circle == CIRCLE_LEFT_IN){
		lcd_showstr(0, 180, "C_LIN ");
       // P32 = 0;
	}
	else if(flag_circle == CIRCLE_LEFT_RUNNING){
		lcd_showstr(0, 180, "C_LRN ");
		//P32 = 0;
	}
	else if(flag_circle == CIRCLE_LEFT_OUT){
		lcd_showstr(0, 180, "C_LOUT");
		//P32 = 1;
	}
	else if(flag_circle == CIRCLE_NONE){
		lcd_showstr(0, 180, "C_NONE ");
    P32 = 0;
	}
	else if(flag_circle == CIRCLE_RIGHT_BEGIN) {
		lcd_showstr(0, 180, "R_BIN ");
		 //P32 = 1;
	}
	else if(flag_circle == CIRCLE_RIGHT_IN){
		lcd_showstr(0, 180, "C_RIN ");
		 //P32 = 0;
	}
	else if(flag_circle == CIRCLE_RIGHT_RUNNING){
		lcd_showstr(0, 180, "C_RRN ");
		 //P32 = 1;
	}
	else if(flag_circle == CIRCLE_RIGHT_OUT){
		lcd_showstr(0, 180, "C_ROUT");
		 //P32 = 0;
	}
			
  //lcd_showuint16(0, 0,count_ones(ccd_data_01,128));
  lcd_show_ccd(ccd_data_01,ccd_data_02);//显示数据 ccd1为高，ccd2低
//			lcd_showuint16(50, 0,count_ones(ccd_data_02,128));
  lcd_showuint8(0, 18,right_point1);
  lcd_showuint8(90, 18,left_point1);
  lcd_showint8(32, 18,offset1);
  // lcd_showuint8(64, 18,right_point1-left_point1);
  lcd_showuint8(64, 18,count_ones_ccd1);
  
  lcd_showuint8(0, 0,Now_right_point2);
  lcd_showuint8(90, 0,Now_left_point2);
  lcd_showint8(32, 0,offset2);
  lcd_showuint8(64, 0,count_ones_ccd2);


  if(key_test==KEY3_PRESS){A += 0.001;}
  if(key_test==KEY4_PRESS){A -= 0.001;}
  //lcd_showfloat(50,120,KP,2,1);
//  lcd_showfloat(70,120,A,1,3);
     //lcd_showuint16(0,120,count_ones(ccd_data_02, 128));
  //lcd_showfloat(0,120,KP,2,3); 
  //lcd_showfloat(0, 120,KP , 2,3);
  //lcd_showfloat(0, 150,A , 1,3);test0
  //lcd_showuint16(60, 150,(zebra_stop(ccd_data_02,128)));
  lcd_showuint16(0, 120,test0);
	//lcd_showuint16(0, 150,dl1b_distance_mm);
  lcd_showfloat(60, 150,yaw_error(circle_begin_yaw,1),3,2);
			// lcd_showuint16(80, 150,ccd1_w_count);
}

//菜单界面1
void display_1(void){
	lcd_showstr(0, 0, "S_P");
  lcd_showfloat(40,0,turn_kp,2,1);
	
  lcd_showstr(0, 17, "S_D");
  lcd_showfloat(40,17,turn_kd,2,1);
	
  lcd_showstr(0, 17*2, "S_I");
  lcd_showfloat(40,17*2,steer_pid.Ki,1,2);

  lcd_showstr(0, 17*3, "tc");
  lcd_showuint16(40,17*3,tiaocang_num);
	
  lcd_showstr(0, 17*4, "M_str");
  lcd_showuint16(40,17*4,speed_stright);

  lcd_showstr(0, 17*5, "M_tur");
  lcd_showuint16(40,17*5,speed_turn);
  
  lcd_showstr(0, 17*6, "M_cro");
  lcd_showuint16(40,17*6,speed_cross);
	
	
  lcd_showstr(0, 17*7, "M_ram");
  lcd_showuint16(40,17*7,speed_ramp);
	
  lcd_showstr(120, key_mode * 17, "<");
	
  
  if(key_test==KEY2_PRESS){
  	lcd_clear(BLACK); //  清屏
    key_mode += 1;
    if(key_mode >= 10)key_mode = 0;
  }

    
		switch (key_mode)
		{
			case 0 :{
				if(key_test==KEY3_PRESS){turn_kp += 1;}//turn_kp 1
				if(key_test==KEY4_PRESS){turn_kp -= 1;}
				break;
			}		
			case 1 :{
				if(key_test==KEY3_PRESS){turn_kd += 2;}
				if(key_test==KEY4_PRESS){turn_kd -= 2;}
				break;
			}
			case 2 :{
				if(key_test==KEY3_PRESS){steer_pid.Ki += 0.01;}
				if(key_test==KEY4_PRESS){steer_pid.Ki -= 0.01;}
				break;
			}
			case 3 :{
				if(key_test==KEY3_PRESS){motor_pid.target_val += 200;}
				if(key_test==KEY4_PRESS){motor_pid.target_val -= 200;}
				break;
			}
			case 4 :{
				if(key_test==KEY3_PRESS){speed_stright += 20;}
				if(key_test==KEY4_PRESS){speed_stright -= 20;}
				break;
			}
			case 5 :{
				if(key_test==KEY3_PRESS){speed_turn += 20;}
				if(key_test==KEY4_PRESS){speed_turn -= 20;}
				break;
			}
			case 6 :{
				if(key_test==KEY3_PRESS){speed_cross += 20;}
				if(key_test==KEY4_PRESS){speed_cross -= 20;}
				break;
			}
			case 7 :{
				if(key_test==KEY3_PRESS){speed_ramp += 20;}
				if(key_test==KEY4_PRESS){speed_ramp -= 20;}
				break;
			}
		}
}

//设置元素检测的先后顺序
void element_sort (void){
  //1
  if(flag_ramp != RAMP_NONE){
    circle_left_big_check = 1;
    ramp_check = 0;
  }
  //2
  else if(flag_circle != CIRCLE_NONE){
    cross_check = 1;
    circle_left_big_check = 0;
  }
  //3
  else if(flag_cross != CROSS_NONE){
    circle_left_check = 1;
    cross_check = 0;
  }
  //4
  else if(flag_circle == CIRCLE_LEFT_OUT && circle_left_out_flag == 2){
    cross_check = 1;
    cross_circle_cleck = 1;
    circle_left_check = 0;
  }
  //5
  else if(flag_cross_circle != CROSS_CIRCLE_NONE){
    cross_check = 0;
    cross_circle_cleck = 0;
    ramp_check = 1;
  }
  

}