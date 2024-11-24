#include "headfile.h"
// uint8 circle_sideline_count = 0;
// uint8 circle_distance = 0;
// uint8 circle_count = 0;  //环岛状态计次
// uint8 circle_left_in_flag = 0;//入环
// uint8 circle_right_in_flag = 0;
// uint8 circle_left_out_flag = 0;//出环
// uint8 circle_right_out_flag = 0;
// uint8 circle_left_out = 0;//出环
// uint8 ccd1_circle_left_out = 0;//出环
// uint8 l_circle_max = 0; //环岛曲线最大点
// int8 last_offset2 = 0;
uint16 distance_saw = 0;
uint16 distance_cross_saw = 0;
float circle_begin_yaw = 0;
float circle_in_yaw = 0;
float cross_circle_yaw = 0;
float circle_yaw_error = 0;
uint8 circle_big_flag = 0;
uint8 circle_small = 0;
uint8 circle_size = 0; //1为小 2为大
int count_big = 0;
int circle_flag_in_cnt=0;
int circle_flag_out_cnt=0;
int tiaocang_num=210;

//十字圆环，过十字后检测是不是圆环后接十字，解决斜入十字无法识别问题
void cross_cricle_proccess(){
	//过了十字后一段路程进行检测
	if(cross_cricle_distence < 16000 && flag_cross_circle == CROSS_CIRCLE_NONE){
		//左环判断
		if(yaw_error(cross_circle_yaw,1) > 140 && yaw_error(cross_circle_yaw,1) < 160 ){
			flag_cross_circle = CROSS_CIRCLE_LEFT_RUNNING;
			//判断为真，刷新里程，用于出环进十字再开始计算
			cross_cricle_distence = 0;
			count_cross_cricle_flag = count_stop;
			test0=1;
		}
		//右环判断
		if(yaw_error(cross_circle_yaw,0) > 140 && yaw_error(cross_circle_yaw,0) < 160 ){
			flag_cross_circle = CROSS_CIRCLE_RIGHT_RUNNING;
			//判断为真，刷新里程，用于出环进十字再开始计算
			cross_cricle_distence = 0;
			count_cross_cricle_flag = count_stop;
			test0=4;
		}
	}
	
///////////////////////////RUNNING////////////////////////////////////
	//左环
	else if (flag_cross_circle == CROSS_CIRCLE_LEFT_RUNNING){
		test0=2;
		if(count_ones_ccd1 > 50 || (yaw_error(cross_circle_yaw,1) > 225 && yaw_error(cross_circle_yaw,1) < 240)){
			//获取出环角度
			get_target_yaw_cross_cricle(0);
			flag_cross_circle = CROSS_CIRCLE_OUT;
			//出环十字，处理类似于正常十字
			cross_cricle_distence = 0;
			count_cross_cricle_flag = count_start;
		}
	}
	//右环
	else if (flag_cross_circle == CROSS_CIRCLE_RIGHT_RUNNING){
		test0 = 5;
		if(count_ones_ccd1 > 50 || (yaw_error(cross_circle_yaw,0) > 225 && yaw_error(cross_circle_yaw,0) < 240)){
			//获取出环角度
			get_target_yaw_cross_cricle(1);
			flag_cross_circle = CROSS_CIRCLE_OUT;
			//出环十字，处理类似于正常十字
			cross_cricle_distence = 0;
			count_cross_cricle_flag = count_start;
		}
	}
			
///////////////////////////OUT////////////////////////////////////
	//出环进十字
	else if (flag_cross_circle == CROSS_CIRCLE_OUT){
		test0 = 3;
		distance_cross_saw = cross_cricle_distence;
		if(cross_cricle_distence > 5000){
			test0=0;
			flag_cross_circle = CROSS_CIRCLE_NONE;
			//防止再次进入十字圆环判断
			cross_cricle_distence = 60000;
		}
	}
	else if (flag_cross_circle != CROSS_CIRCLE_NONE && flag_cross != CROSS_NONE){

		flag_cross_circle = CROSS_CIRCLE_NONE;
		test0=0;
		//防止再次进入十字圆环判断
		cross_cricle_distence = 60000;
	}
}


void cricle_proccess()
{
//////////////////////////////////////////////////////////////左圆环////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////左圆环////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////左圆环////////////////////////////////////////////////////////////
	//状态1:赛道宽变大 左边线消失 右边线还在
	if(flag_cross_circle == CROSS_CIRCLE_NONE && flag_circle == CIRCLE_NONE && flag_cross == CROSS_NONE && flag_ramp == RAMP_NONE
		&& count_ones_ccd2 > 80 && count_ones_ccd1 <40 && count_ones_ccd2 < 100   //赛道宽度条件
		&& left_point2 < 15 && right_point2 < 100                                         //赛道边界条件
		&& (Now_right_point2 - right_point1) < 30                                         //右边是直线
		&& (circle_no_distence >= 39000)                                                 //二次圆环
		&& circle_left_check){

			count_crbegin_flag = count_start;
			flag_circle = CIRCLE_LEFT_BEGIN;
			circle_count = 0;
	}

	//状态2：处于丢线处
	else if(flag_circle == CIRCLE_LEFT_BEGIN) 
	{
//		if(/*误识别圆环判断*/ 0){
//			circle_begin_yaw = 0;	
//			flag_circle = CIRCLE_NONE;
//			circle_count = 0;
//		}
		
		circle_in_yaw = yaw;

	//用count_ones_ccd1 代替 count_ones(ccd_data_01,128)节省算力
		if(count_ones_ccd1 > 45)
		{
			//记录begin状态的yaw，出环角度判断
			circle_begin_yaw = yaw;		
			flag_circle = CIRCLE_LEFT_IN; 
			distance_saw = circle_begin_distence; //记录bigin到进入in状态那一刻的路程 用于判断大圆环小圆环
			count_crbegin_flag = count_stop;
		}
		
		left_point2 = right_point2 - 53;  //没进入下一个赛道前，左边线等于右边线减赛道宽
	}
	//状态3：处于准备进入环岛的地方 
	else if(flag_circle == CIRCLE_LEFT_IN) {

//		offset_keep_flag = 1;

		circle_yaw_error = yaw_error(circle_begin_yaw,1);
		//转到一定角度进入下一状态
		
		if(distance_saw <= 45000){offset_keep = 16;offset_keep_flag = 1;}
		else if(distance_saw > 45000)right_point2 = left_point2 + 15;  //切成左巡线
		
		
		if(yaw_error(circle_begin_yaw,1)>60 && yaw_error(circle_begin_yaw,1)<100){ 
			flag_circle = CIRCLE_LEFT_RUNNING;
			offset_keep = 0;
			offset_keep_flag = 0;
			
		}	
		
		

	}
	
/////////////////////////////////////CIRCLE_LEFT_RUNNING/////////////////////////////////////
	
	//状态6：环岛内 （先写死了 具体offset值在下面get_offset函数）	
	else if(flag_circle == CIRCLE_LEFT_RUNNING) {
			
		circle_yaw_error = yaw_error(circle_begin_yaw,1);
//		//情况2：转到一定角度出环
		
			if((right_point1 > 100 || right_point2 > 100 )&& circle_left_out_flag == 0){
			
			if(distance_saw <= 45000){
				if(yaw_error(circle_begin_yaw,1)>280){ 
					offset_keep_flag = 0;
					
					flag_circle = CIRCLE_LEFT_OUT;
				}
			}else {
				
				if(yaw_error(circle_begin_yaw,1)>300){ 
					left_point2 = right_point2 - 35; 
					offset_keep_flag = 0;
					
					flag_circle = CIRCLE_LEFT_OUT;
				}
			}
	
	
		}
	}
/////////////////////////////////////CIRCLE_LEFT_OUT/////////////////////////////////////
	//状态6：出环岛 （先不处理）
	else if(flag_circle == CIRCLE_LEFT_OUT){//用惯导


		circle_yaw_error = yaw_error(circle_begin_yaw,1);
		target_yaw = circle_begin_yaw;
		offset_keep_flag = 0;
//		if((right_point1 > 100 || right_point2 > 100 )&& circle_left_out_flag == 0){
//			
			//丢右线状态，固定打角，用running状态的平均打角效果更好
//			if(distance_saw <= 45000 && (yaw_error(circle_begin_yaw,1)>260)){offset_keep = 18;offset_keep_flag = 0;}        //小环打脚
//			else if(distance_saw > 45000 && (yaw_error(circle_begin_yaw,1)>285)){offset_keep = 12;offset_keep_flag = 0;}      //大环打脚
			// if(circle_left_check)offset_keep = 17;           //小环打脚
			// else if(circle_left_big_check)offset_keep = 12;       //大环打脚
				//P32 = 1;
				test0 = 2;
		
	
			if((yaw_error(circle_begin_yaw,1)>330 || yaw_error(circle_begin_yaw,1) < 20) && distance_saw <= 45000 /*&& count_ones_ccd1 > 25 && count_ones_ccd2 > 70*/)circle_left_out_flag = 2;   //分开大环小环判条件

			if((yaw_error(circle_begin_yaw,1)>330 || yaw_error(circle_begin_yaw,1) < 20) && distance_saw > 45000 )circle_left_out_flag = 2;			
//			
		//}

		//回到与圆心平齐的位置，
		//if(circle_left_out_flag == 2){
		
//				P32 = 1;
//				//重新找到线，不用打死了
//				offset_keep = 0;
//				offset_keep_flag = 0;

//				circle_count = 0;
//				last_left_point2 = 0;		
//				circle_left_in_flag = 0;	
//				
//				left_point2 = right_point2 - 50;  //左边线等于右边线减赛道宽 
//				test0 = 3;	
//			//恢复普通赛道，离开环岛
			if((count_ones_ccd2 < 55 && Now_left_point2 > 20 && circle_left_out_flag == 2 && distance_saw <= 45000) ||
				(count_ones_ccd2 < 55 && Now_left_point2 > 20 && count_ones_ccd1 < 40 && circle_left_out_flag == 2 && distance_saw > 45000)){
				P32 = 1;
				//重新找到线，不用打死了
				offset_keep = 0;
				offset_keep_flag = 0;
				count_nocri_flag = count_start;
				circle_count = 0;          //各种标志位全部清零
				last_left_point2 = 0;		
				circle_left_in_flag = 0;
				circle_left_out_flag = 0;
				flag_circle = CIRCLE_NONE; //恢复可检测的初始状态
				circle_begin_distence = 0;
				distance_saw = 0;
				test0 = 4;
			}
			
		//}
		if( (Now_right_point2-Now_left_point2) > 48 && (Now_right_point2-Now_left_point2)<=54 
				&& offset2 > -22 && offset2 <22 ){
					P32 = 0;
					//重新找到线，不用打死了
					offset_keep = 0;
					offset_keep_flag = 0;
					count_nocri_flag = count_start;
					circle_count = 0;          //各种标志位全部清零
					last_left_point2 = 0;		
					circle_left_in_flag = 0;
					circle_left_out_flag = 0;
					flag_circle = CIRCLE_NONE; //恢复可检测的初始状态
					circle_begin_distence = 0;
					distance_saw = 0;
					test0 = 4;
				}

	}
	
//////////////////////////////////////////////////////////////右圆环////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////右圆环////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////右圆环////////////////////////////////////////////////////////////
	//状态1:赛道宽变大 右边线消失 左边线还在
	if(flag_cross_circle == CROSS_CIRCLE_NONE && flag_circle == CIRCLE_NONE && flag_cross == CROSS_NONE && flag_ramp == RAMP_NONE
		&& count_ones_ccd2 > 80 && count_ones_ccd1 <40      //赛道宽度条件
		&& left_point2 > 20 && right_point2 > 100                                         //赛道边界条件
		&& (myabs(Now_left_point2 - left_point1)) < 30                                         //右边是直线
		/*&& (circle_no_distence >= 39000)*/                                                  //二次圆环
		&& 1){

			count_crbegin_flag = count_start;
			flag_circle = CIRCLE_RIGHT_BEGIN;
			circle_count = 0;

	}

	//状态2：处于丢线处
	else if(flag_circle == CIRCLE_RIGHT_BEGIN) {

//		if(/*误识别圆环判断*/ 0){
//			circle_begin_yaw = 0;	
//			flag_circle = CIRCLE_NONE;
//			circle_count = 0;
//		}
		
		circle_in_yaw = yaw;

	//用count_ones_ccd1 代替 count_ones(ccd_data_01,128)节省算力
		if(count_ones_ccd1 > 45)
		{

			//记录begin状态的yaw，出环角度判断
			circle_begin_yaw = yaw;		
			flag_circle = CIRCLE_RIGHT_IN; 
			distance_saw = circle_begin_distence; //记录bigin到进入in状态那一刻的路程 用于判断大圆环小圆环
			count_crbegin_flag = count_stop;
			
		}
		
		right_point2 = left_point2 + 53;  //没进入下一个赛道前，右边线等于左边线加赛道宽
	}
	//状态3：处于准备进入环岛的地方 
	else if(flag_circle == CIRCLE_RIGHT_IN) {
		//丢右线状态，固定打角，用running状态的平均打角效果更好
//		
//		offset_keep_flag = 1;

	    circle_yaw_error = yaw_error(circle_begin_yaw,1);
		//转到一定角度进入下一状态

		if(yaw_error(circle_begin_yaw,0)>60 && yaw_error(circle_begin_yaw,0)<100){ 
			flag_circle = CIRCLE_RIGHT_RUNNING;
			offset_keep = 0;
			offset_keep_flag = 0;
			
		}	
		
		left_point2 = right_point2 - 53;  //切成右巡线

		
	}
	
/////////////////////////////////////CIRCLE_RIGHT_RUNNING/////////////////////////////////////
	
	//状态6：环岛内 （先写死了 具体offset值在下面get_offset函数）	
	else if(flag_circle == CIRCLE_RIGHT_RUNNING) {
			
	    circle_yaw_error = yaw_error(circle_begin_yaw,0);
//		//情况2：转到一定角度出环

		if(yaw_error(circle_begin_yaw,0)>220){ 
			
			offset_keep_flag = 0;

			flag_circle = CIRCLE_RIGHT_OUT;
		}
		right_point2 =left_point2  + 58;  //切成右巡线

	}
/////////////////////////////////////CIRCLE_RIGHT_OUT/////////////////////////////////////
	//状态6：出环岛 （先不处理）
	else if(flag_circle == CIRCLE_RIGHT_OUT){


		circle_yaw_error = yaw_error(circle_begin_yaw,0);
		
		if(left_point2 < 28 && circle_right_out_flag == 0){
			
			//丢右线状态，固定打角，用running状态的平均打角效果更好
			if(distance_saw <= 45000)offset_keep = -16;           //小环打脚
			else if(distance_saw > 45000)offset_keep = -12;       //大环打脚
			
			offset_keep_flag = 1;
			circle_right_out_flag = 1 ;  //	确认左丢线
		}
		if(circle_right_out_flag == 1)
		{
			P32=1;
//			circle_flag_out_cnt++;
//			if(count_ones_ccd1 > 25 && count_ones_ccd2 > 70 )
//			{
				if((yaw_error(circle_begin_yaw,0)>350 || yaw_error(circle_begin_yaw,0) < 20) && distance_saw <= 45000)circle_right_out_flag = 2;   //分开大环小环判条件
				if((yaw_error(circle_begin_yaw,0)>340 || yaw_error(circle_begin_yaw,0) < 20) && distance_saw > 45000)circle_right_out_flag = 2;
//			}
			
//			if(circle_flag_out_cnt>=30) 	circle_right_out_flag=2;
				
		}
		//回到与圆心平齐的位置，
		else if(circle_right_out_flag == 2)
		{
			P32=0;	
			//重新找到线，不用打死了
			offset_keep = 0;
			offset_keep_flag = 0;

			circle_count = 0;
			last_right_point2 = 0;		
			circle_right_in_flag = 0;	
			
			right_point2 = left_point2 + 53;  //右边线等于左边线加赛道宽 
			
			if(count_ones_ccd1 < 35)circle_count = 1;   //调的时候发现环岛状态会提前结束，多设置了个标志位
			
			//恢复普通赛道，离开环岛
			if(ccd2_w_count < 53 && right_point2 < 108 && circle_count == 1)
			{
				//重新找到线，不用打死了
				offset_keep = 0;
				offset_keep_flag = 0;
				count_nocri_flag = count_start;
				circle_count = 0;          //各种标志位全部清零
				last_right_point2 = 0;		
				circle_right_in_flag = 0;
				circle_right_out_flag = 0;
				flag_circle = CIRCLE_NONE; //恢复可检测的初始状态
				circle_begin_distence = 0;
				distance_saw = 0;
				circle_flag_in_cnt=0;
				circle_flag_out_cnt=0;
			}
		}

	}
}
