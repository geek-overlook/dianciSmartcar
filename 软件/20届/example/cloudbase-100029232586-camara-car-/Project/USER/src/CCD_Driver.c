#include "CCD_Driver.h"

uint16 ccd_data_01[128] = {0};
uint16 ccd_data_02[128] = {0};
//最大白线宽度
uint8 count_ones_ccd1 = 0;
uint8 count_ones_ccd2 = 0;
uint8 last_count_ones_ccd1 = 0;
uint8 last_count_ones_ccd2 = 0;
uint8  ccd1_l_loss = 0;
uint8  ccd1_r_loss = 0;
uint8  ccd2_l_loss = 0;
uint8  ccd2_r_loss = 0;
uint8 circle_sideline_count = 0;
uint8 circle_distance = 0;
uint8 circle_count = 0;  //环岛状态计次
uint8 circle_left_in_flag = 0;//入环
uint8 circle_right_in_flag = 0;
uint8 circle_left_out_flag = 0;//出环
uint8 circle_right_out_flag = 0;
uint8 circle_left_out = 0;//出环
uint8 ccd1_circle_left_out = 0;//出环


uint16 ccd1_w_count = 0;
uint16 ccd2_w_count = 0;
uint8 ccd_select = 0;
uint8 l_circle_max = 0; //环岛曲线最大点
uint8 last_left_point1 = 0;  //上一次的左边线点
uint8 last_right_point1 = 0;  //上一次的右边线点
uint8 last_left_point2 = 0;  //上一次的左边线点
uint8 last_right_point2 = 0;  //上一次的右边线点
int8 offset1=0;//误差
int8 offset2=0;//误差
int8 last_offset=0;//上一次偏差值
int8 offset_inertia=0;//编码器计算的误差
float aim_angle_filter = 0;
int8 last_offset2 = 0;
uint8 zebra_count_stop =0;
uint16 ccd_value1 = 0;
uint16 ccd_value2 = 0;
uint8 Now_left_point2 = 0;
uint8 Now_right_point2 = 0;
uint8 tof_enable = 1;
int8 offset_weight = 0;
uint16 element_ccd_value = 0;


////////////////////////////////////控制////////////////////////////////////
//分段pid
uint32 inertia_kp = 5;
uint32 inertia_kd = 10;
uint32 turn_kp = 9;
uint32 turn_kd = 30;
//速度
uint16 speed_turn = 840;//700
uint16 speed_cross = 800;//700
uint16 speed_ramp = 600;
uint16 speed_stright = 1150;//1000
int16  limit_motor_pwm = 0;


/////////////////////////////////元素检测标志位//////////////////////////////
uint8 circle_left_check = 0 ;
uint8 circle_left_big_check = 0 ;
uint8 circle_right_check = 0 ;
uint8 cross_check = 0 ;
uint8 cross_circle_cleck = 0;
uint8 ramp_check = 0 ;
uint8 zebra_flag = 0 ;//置1才开始检测斑马线并停车

enum flag_circle_e flag_circle=0;
enum flag_cross_circle_e flag_cross_circle=0;
enum flag_ramp_e flag_ramp = 0;
enum flag_cross_e flag_cross = 0;

uint16 ccd_threshold(uint16 ccd_data[])
{
	uint16 i;
	uint16 value1_max, value1_min;
	uint16 CCD_Threshold = 0;

    value1_max = ccd_data[0];      // 动态阈值算法，读取最大和最小值
    for (i = 5; i < 123; i++) // 两边各去掉5个点
    {
        if (value1_max <= ccd_data[i])
            value1_max = ccd_data[i];
    }
    value1_min = ccd_data[0]; // 最小值
    for (i = 5; i < 123; i++)
    {
        if (value1_min >= ccd_data[i])
            value1_min = ccd_data[i];
    }
    CCD_Threshold = (value1_max + value1_min) / 2; // 计算出本次中线提取的阈值	
	return CCD_Threshold;
}

//计算ccd突变点数量，检测斑马线
uint16 zebra_stop(uint16 arr[], uint16 n) {
	int8 i;
	uint16 count = 0;
	uint16 prev = arr[0]; // 记录上一个元素的值	
	if (n <= 0) {
		return 0; // 数组长度必须大于0
	}
	for ( i = 1; i < n; ++i) {
		if (arr[i] != prev) {
			count++; // 遇到跳变则计数加1
		}
		prev = arr[i];
	}

	return count;
}



void ccd_two_value(uint16 value1, uint16 value2)//ccd数据二值化  黑点为0，白点为1   value 300左右
{
	uint8 i=0;
	uint16 temp_num1=0;
	uint16 temp_num2=0;
	ccd1_w_count = 0;
    ccd2_w_count = 0;
	
	for( i=0;i<128;i++)
	{
		//2:ccd_data_ch2[i] 1:
		temp_num1=ccd_data_ch1[i];
	    temp_num2=ccd_data_ch2[i];
		
		if(temp_num1>value1){ccd_data_01[i]=1;ccd1_w_count++;}//白点为1
		else               {ccd_data_01[i]=0;}//黑点为0
		
		if(temp_num2>value2){ccd_data_02[i]=1;ccd2_w_count++;}//白点为1
		else               {ccd_data_02[i]=0;}//黑点为0
	}
}

//【2】点集滤波
void Bin_CCD_Filter()
{
	uint16 i = 0;
	for(i = 1; i < ccd1_w_count; i++)
	{
		if(ccd_data_01[i] == 1 && ccd_data_01[i-1] == 0 && ccd_data_01[i+1] == 0)ccd_data_01[i] = 0;
		else if(ccd_data_01[i] == 0 && ccd_data_01[i-1] == 1 && ccd_data_01[i+1] == 1)ccd_data_01[i] = 1;
	}
	
	for(i = 1; i < ccd2_w_count; i++)
	{
		if(ccd_data_02[i] == 1 && ccd_data_02[i-1] == 0 && ccd_data_02[i+1] == 0)ccd_data_02[i] = 0;
		else if(ccd_data_02[i] == 0 && ccd_data_02[i-1] == 1 && ccd_data_02[i+1] == 1)ccd_data_02[i] = 1;
	}	
}

uint16 myabs(int16 number)//绝对值函数
{
	static uint16 abs_number=0;
	 
	if( number>=0)abs_number=number;
	else abs_number=-number;

	return abs_number;

}

uint8 left_point1 =   0;//ccd1左边点
uint8 right_point1= 127;//ccd1右边点
uint8 mid_point1  =  64;//ccd1中点

uint8 left_point2 =   0;//ccd2左边点
uint8 right_point2= 127;//ccd2右边点
uint8 mid_point2  =  64;//ccd2中点
uint8 last_mid_point2=64;

void get_mid_point1(void)//计算ccd1得中点
 {
	static uint8 last_mid_point1=64;
	uint8 l_point1=0;
	uint8 r_point1=0;
	ccd1_circle_left_out = 0;
	for( l_point1=last_mid_point1;l_point1>0;l_point1--)//搜索左边点，以上次中点作为这次的起搜点
	{
		if(ccd_data_01[l_point1-1]==0&&ccd_data_01[l_point1]==0)//找到连续两个黑点
		{
			left_point1=l_point1;
			break;//找到左边点退出循环
			}
			else if(l_point1==1)//找不到左边点
			{
				ccd1_l_loss++;
				left_point1=0;//以0作为左边点
			break;//退出循环
			}
	}
	for( r_point1=last_mid_point1;r_point1<127;r_point1++)//搜索右边点，以上次中点作为这次的起搜点
	{
		if(ccd_data_01[r_point1+1]==0&&ccd_data_01[r_point1]==0)//找到连续两个黑点
			{
				right_point1=r_point1;
				break;//找到右边点退出循环
			}
		else if(r_point1==126)//找不到右边点
			{
				ccd1_r_loss++;
			right_point1=127;//以127作为右边点
			break;//退出循环
			}
		}


	// //找线失败，进入第二次找线
	// if((right_point1-left_point1)<=1 && count_ones_ccd1>5 ){
	// 	//情况1：白线只有一段
	// 	if(zebra_stop(ccd_data_01,128) <= 2){
	// 		//找左线
	// 		//白线在最左边
	// 		if(ccd_data_01[0]==1){
	// 			left_point1 = 0;
	// 		}
	// 		//白线不在最左边，找黑变白的点
	// 		else{
	// 			for(l_point1 = 1 ; l_point1<128 ; l_point1++){
	// 				if(ccd_data_01[l_point1-1]==0 && ccd_data_01[l_point1]==1){
	// 					left_point1 = l_point1;
	// 					break;
	// 				}
	// 			}
	// 		}

	// 		//找右线
	// 		if(ccd_data_01[127]==1){
	// 			right_point1 = 127;
	// 		}
	// 		//白线不在最右边，找黑变白的点
	// 		else{
	// 			for(r_point1 = 126 ; r_point1>0 ; r_point1--){
	// 				if(ccd_data_01[r_point1+1]==0 && ccd_data_01[r_point1]==1){
	// 					right_point1 = r_point1;
	// 					break;
	// 				}
	// 			}
	// 		}
	// 	}
	// }
	

	
        //    LimitL(left_point1);//左边点限幅
		//    LimitH(right_point1);//右边点限幅



		//消除进圆环的前摇,只能消除直道圆环
		if((Now_right_point2 -right_point1 >5) && (Now_right_point2 -right_point1 <15) && //右边为直线
		((count_ones_ccd1 < 55 && count_ones_ccd2 > 75 )||//ccd2丢线 ccd1不丢线
		(count_ones_ccd1 > 68 && count_ones_ccd2 < 53 ))&&//ccd1丢线 ccd2不丢线
		flag_ramp == RAMP_NONE && flag_cross_circle == CROSS_CIRCLE_NONE && flag_circle == CIRCLE_NONE && flag_cross == CROSS_NONE//不在任何一个元素
		){
			left_point1 = right_point1 - 30;
		}

		if((left_point1 - Now_left_point2 >5) && (left_point1 - Now_left_point2 <15) && //左边为直线
		(count_ones_ccd1 < 55 && count_ones_ccd2 > 75 )||//ccd2丢线 ccd1不丢线
		(count_ones_ccd1 > 68 && count_ones_ccd2 < 53 )&&//ccd1丢线 ccd2不丢线
		flag_ramp == RAMP_NONE && flag_cross_circle == CROSS_CIRCLE_NONE && flag_circle == CIRCLE_NONE && flag_cross == CROSS_NONE//不在任何一个元素
		){
			right_point1 = left_point1 + 30;
		}
    
		if(myabs(last_mid_point1- mid_point1)<20)
		{
			mid_point1=(left_point1+right_point1)/2;
		
		}
		
		else//过十字时防止因为车身摆动导致误差过大误差过大，以上次中点作为这次中点
			
		{
		mid_point1=last_mid_point1;//过十字时防止因为车身摆动导致误差过大，以上次中点作为这次中点
		
		}
		
		if(myabs(last_mid_point1- mid_point1) == 127)ccd1_circle_left_out = 1;
    last_mid_point1= mid_point1;//以这次中点作为下次的上次中点，并作为起搜点
}

void get_mid_point2(void)//计算ccd2得中点
{

	uint8 l_point2=0;
	uint8 r_point2=0;
	static uint8 running_count;
	//搜索左边点，以上次中点作为这次的起搜点
	for( l_point2=last_mid_point2;l_point2>0;l_point2--){
		//找到连续两个黑点
		if(ccd_data_02[l_point2-1]==0&&ccd_data_02[l_point2]==0){
			left_point2=l_point2;
			break;//找到左边点退出循环
		}
		//找不到左边点
		else if(l_point2==7){
			left_point2=0;//以0作为左边点
			ccd2_l_loss++;
			break;//退出循环
		}
	}
	//搜索右边点，以上次中点作为这次的起搜点
	for( r_point2=last_mid_point2;r_point2<127;r_point2++){
		//找到连续两个黑点
		if(ccd_data_02[r_point2+1]==0&&ccd_data_02[r_point2]==0){
			right_point2=r_point2;
			break;//找到右边点退出循环
		}
		//找不到右边点
		else if(r_point2==120){
			right_point2=120;//以127作为右边点
			ccd2_r_loss++ ;
			break;//退出循环
		}
	}
	Now_left_point2 = left_point2;
	Now_right_point2 = right_point2;

	//圆环
	cricle_proccess();

	//消除进圆环的前摇,只能消除直道圆环
	if((Now_right_point2 -right_point1 >5) && (Now_right_point2 -right_point1 <15) && //右边为直线
	((count_ones_ccd1 < 55 && count_ones_ccd2 > 75 )||//ccd2丢线 ccd1不丢线
	(count_ones_ccd1 > 68 && count_ones_ccd2 < 53 ))&&//ccd1丢线 ccd2不丢线
	flag_ramp == RAMP_NONE && flag_cross_circle == CROSS_CIRCLE_NONE && flag_circle == CIRCLE_NONE && flag_cross == CROSS_NONE//不在任何一个元素
	){
		left_point2 = right_point2 - 51;
	}

	if((left_point1 - Now_left_point2 >5) && (left_point1 - Now_left_point2 <15) && //右边为直线
	((count_ones_ccd1 < 55 && count_ones_ccd2 > 75 )||//ccd2丢线 ccd1不丢线
	(count_ones_ccd1 > 68 && count_ones_ccd2 < 53 ))&&//ccd1丢线 ccd2不丢线
	flag_ramp == RAMP_NONE && flag_cross_circle == CROSS_CIRCLE_NONE && flag_circle == CIRCLE_NONE && flag_cross == CROSS_NONE//不在任何一个元素
	){
		right_point2 = left_point2 + 51;
	}

	

	last_right_point2 = right_point2;
	LimitL(left_point2);//左边点限幅
	LimitH(right_point2);//右边点限幅


	if(myabs(last_mid_point2- mid_point2)<20){
		mid_point2=(left_point2+right_point2)/2;
	}
	else{//过十字时防止因为车身摆动导致误差过大误差过大，以上次中点作为这次中点
		mid_point2=last_mid_point2;//过十字时防止因为车身摆动导致误差过大，以上次中点作为这次中点
	}
    last_mid_point2= mid_point2;//以这次中点作为下次的上次中点，并作为起搜点
}


//计算误差
void get_offset(void){

	last_offset = offset2;
	offset1 = 64 - mid_point1;
	offset2 = 64 - mid_point2;
	if(((offset2 > 0 && offset1 < 0) || (offset2 < 0 && offset1 > 0)) && count_ones_ccd1 > 34){
		offset_weight = offset1*0.6 + offset2*0.1;
	}else{
		offset_weight = offset1*0.4 + offset2*0.7;
	}
	
	KP_get(offset_weight);
	
	//if(flag_ramp != RAMP_NONE && flag_circle == CIRCLE_NONE){offset2 = 0;}
//	if(circle_left_out_flag == 1 )offset2 = calc_ave(circle_run_point,20);
	if(!process_flag){
		motor_pwm_out = 0;//丢线保护
		motor_pid.target_val = (float)0;
	}else if(0){//惯导测试
		motor_pid.target_val = (float)440;
		steer_pid.Kp=inertia_kp;
		steer_pid.Kd = inertia_kd;
	}
	else if(flag_circle == CIRCLE_LEFT_BEGIN || flag_circle == CIRCLE_RIGHT_BEGIN)
	{
		motor_pid.target_val = (float)780;
		steer_pid.Kp=5;
		steer_pid.Kd = 17;
	}
	//小圆环环内
	else if((flag_circle == CIRCLE_LEFT_RUNNING || flag_circle == CIRCLE_RIGHT_RUNNING) && distance_saw <= 45000){
		motor_pid.target_val = (float)900;
		steer_pid.Kp= turn_kp;				//steer_pid.Kp=KP;
		steer_pid.Kd = turn_kd ;			//steer_pid.Kd = KP *3;
		
	}
	//大圆环环内
	else if((flag_circle == CIRCLE_LEFT_IN || flag_circle == CIRCLE_LEFT_RUNNING || flag_circle == CIRCLE_RIGHT_RUNNING) && distance_saw > 45000){
		motor_pid.target_val = (float)920;
		steer_pid.Kp=5;
		steer_pid.Kd = 17;
	}
	//坡道
	else if(flag_ramp != RAMP_NONE){
		motor_pid.target_val = (float)speed_ramp;
		steer_pid.Kp = inertia_kp;
		steer_pid.Kd = inertia_kd;
		// P32 = 1;
	}
	//十字
	else if(flag_cross == CROSS_RUNNING){
		motor_pid.target_val = (float)speed_stright;
//		// P32 = 1;
	}
	else if(flag_cross == CROSS_MID){
		motor_pid.target_val = (float)speed_stright;
		steer_pid.Kp = inertia_kp;
		steer_pid.Kd = inertia_kd;
//		P32 = 0;
	}
	//直道
	else if( (Now_right_point2-Now_left_point2) > 48 && (Now_right_point2-Now_left_point2)<=54 
	&& offset2 > -22 && offset2 <22 
	/*&&flag_circle == CIRCLE_NONE*/ && flag_ramp == RAMP_NONE){
		motor_pid.target_val = (float)speed_stright;
		steer_pid.Kp=5;
		steer_pid.Kd = 17;
//		P32 = 0;
	}
	//高速弯道
	else if(offset2 > 25 || offset2 < -25 ){
		motor_pid.target_val = (float)750;
		steer_pid.Kp=KP;
		steer_pid.Kd=KP *3.2;
//		P32 = 0;
	}
	//弯道
	else{
		motor_pid.target_val = (float)speed_turn;
		steer_pid.Kp=KP;
		steer_pid.Kd=KP *3.2;
//		P32 = 0;
	}



//////////////////////////巡线方式切换//////////////////////////
	if(
		flag_circle != CIRCLE_NONE || flag_cross == CROSS_NONE 
		//不是打固定角度和惯导情况
		&& flag_ramp != RAMP_NONE && !offset_keep_flag
		&& flag_cross_circle != CROSS_CIRCLE_OUT
		){
		aim_angle_filter = offset2;
		// P32 = 0;
	}
	else 
	{
		//ccd1丢线了用纯ccd2寻迹
		if(right_point1-left_point1 == 0){
			aim_angle_filter = offset2*1.2;
			// P32 = 0;
		}
		else {
			// P32 = 1;
			aim_angle_filter = offset_weight;
		}
	}
	//需要用惯导的状态放在这里
	if(flag_ramp == RAMP_IN_MID || flag_cross_circle == CROSS_CIRCLE_OUT || flag_circle == CIRCLE_LEFT_OUT){
		aim_angle_filter = offset_inertia;
	}
	

	//十字
	if(flag_cross != CROSS_NONE  && flag_ramp == RAMP_NONE && !offset_keep_flag){
		//全丢线，惯导
		if(count_ones(ccd_data_01,128)>89&&count_ones(ccd_data_02,128)>=60){
			flag_cross = CROSS_MID;
			aim_angle_filter = offset_inertia;
			// P32 = 1;
		}
		//ccd1有线，ccd2丢线，寻远线
		else if(count_ones(ccd_data_01,128)<40&&count_ones(ccd_data_02,128)>=60 && offset1 < 12 && offset1 > -12){
			flag_cross = CROSS_MID;
			aim_angle_filter = offset_inertia;
			// P32 = 1;
		}
		//ccd1丢线，ccd2未丢线，寻近线
		else if(count_ones(ccd_data_01,128)>89&&count_ones(ccd_data_02,128)<60){
			
			flag_cross = CROSS_RUNNING;
			aim_angle_filter = offset2;
			get_target_yaw(offset2);
			// P32 = 0;
		}
		// ccd1和ccd2都有线，退出十字
		else if(count_ones(ccd_data_01,128)<35&&count_ones(ccd_data_02,128)<=55){
			aim_angle_filter = offset2;
			flag_cross = CROSS_NONE;
		}
//		else if(cross_distence > 27300){                     //这段加上 不知道为什么会在s弯接下来的十字那里概率识别成圆环
//			aim_angle_filter = offset2;
//			flag_cross = CROSS_NONE;                     
//			P32 = 1;
//		}
		else{
			aim_angle_filter = offset_inertia;
		}
	}
	//特殊位置打固定角度
	if(offset_keep_flag){
		aim_angle_filter = offset_keep;
	}
	//斑马线检测
	if(zebra_stop(ccd_data_02,128) >= 7 && count_ones_ccd1 >28 && count_ones_ccd1 < 32){//第一次检测到斑马线
		if(count_ones(ccd_data_02, 128)<7)
			zebra_flag = 1;
			count_zebra_flag = count_start;
	}
	if(zebra_stop(ccd_data_02,128) >= 7 && count_ones_ccd1 >28 && count_ones_ccd1 < 32 && zebra_distence > 50000 && zebra_flag){//第一次检测到斑马线
		if(count_ones(ccd_data_02, 128)<7){
			process_flag = 0;
		}
	}
	
}



