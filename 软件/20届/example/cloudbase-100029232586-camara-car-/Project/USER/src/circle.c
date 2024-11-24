#include "headfile.h"
// uint8 circle_sideline_count = 0;
// uint8 circle_distance = 0;
// uint8 circle_count = 0;  //����״̬�ƴ�
// uint8 circle_left_in_flag = 0;//�뻷
// uint8 circle_right_in_flag = 0;
// uint8 circle_left_out_flag = 0;//����
// uint8 circle_right_out_flag = 0;
// uint8 circle_left_out = 0;//����
// uint8 ccd1_circle_left_out = 0;//����
// uint8 l_circle_max = 0; //������������
// int8 last_offset2 = 0;
uint16 distance_saw = 0;
uint16 distance_cross_saw = 0;
float circle_begin_yaw = 0;
float circle_in_yaw = 0;
float cross_circle_yaw = 0;
float circle_yaw_error = 0;
uint8 circle_big_flag = 0;
uint8 circle_small = 0;
uint8 circle_size = 0; //1ΪС 2Ϊ��
int count_big = 0;
int circle_flag_in_cnt=0;
int circle_flag_out_cnt=0;
int tiaocang_num=210;

//ʮ��Բ������ʮ�ֺ����ǲ���Բ�����ʮ�֣����б��ʮ���޷�ʶ������
void cross_cricle_proccess(){
	//����ʮ�ֺ�һ��·�̽��м��
	if(cross_cricle_distence < 16000 && flag_cross_circle == CROSS_CIRCLE_NONE){
		//���ж�
		if(yaw_error(cross_circle_yaw,1) > 140 && yaw_error(cross_circle_yaw,1) < 160 ){
			flag_cross_circle = CROSS_CIRCLE_LEFT_RUNNING;
			//�ж�Ϊ�棬ˢ����̣����ڳ�����ʮ���ٿ�ʼ����
			cross_cricle_distence = 0;
			count_cross_cricle_flag = count_stop;
			test0=1;
		}
		//�һ��ж�
		if(yaw_error(cross_circle_yaw,0) > 140 && yaw_error(cross_circle_yaw,0) < 160 ){
			flag_cross_circle = CROSS_CIRCLE_RIGHT_RUNNING;
			//�ж�Ϊ�棬ˢ����̣����ڳ�����ʮ���ٿ�ʼ����
			cross_cricle_distence = 0;
			count_cross_cricle_flag = count_stop;
			test0=4;
		}
	}
	
///////////////////////////RUNNING////////////////////////////////////
	//��
	else if (flag_cross_circle == CROSS_CIRCLE_LEFT_RUNNING){
		test0=2;
		if(count_ones_ccd1 > 50 || (yaw_error(cross_circle_yaw,1) > 225 && yaw_error(cross_circle_yaw,1) < 240)){
			//��ȡ�����Ƕ�
			get_target_yaw_cross_cricle(0);
			flag_cross_circle = CROSS_CIRCLE_OUT;
			//����ʮ�֣���������������ʮ��
			cross_cricle_distence = 0;
			count_cross_cricle_flag = count_start;
		}
	}
	//�һ�
	else if (flag_cross_circle == CROSS_CIRCLE_RIGHT_RUNNING){
		test0 = 5;
		if(count_ones_ccd1 > 50 || (yaw_error(cross_circle_yaw,0) > 225 && yaw_error(cross_circle_yaw,0) < 240)){
			//��ȡ�����Ƕ�
			get_target_yaw_cross_cricle(1);
			flag_cross_circle = CROSS_CIRCLE_OUT;
			//����ʮ�֣���������������ʮ��
			cross_cricle_distence = 0;
			count_cross_cricle_flag = count_start;
		}
	}
			
///////////////////////////OUT////////////////////////////////////
	//������ʮ��
	else if (flag_cross_circle == CROSS_CIRCLE_OUT){
		test0 = 3;
		distance_cross_saw = cross_cricle_distence;
		if(cross_cricle_distence > 5000){
			test0=0;
			flag_cross_circle = CROSS_CIRCLE_NONE;
			//��ֹ�ٴν���ʮ��Բ���ж�
			cross_cricle_distence = 60000;
		}
	}
	else if (flag_cross_circle != CROSS_CIRCLE_NONE && flag_cross != CROSS_NONE){

		flag_cross_circle = CROSS_CIRCLE_NONE;
		test0=0;
		//��ֹ�ٴν���ʮ��Բ���ж�
		cross_cricle_distence = 60000;
	}
}


void cricle_proccess()
{
//////////////////////////////////////////////////////////////��Բ��////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////��Բ��////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////��Բ��////////////////////////////////////////////////////////////
	//״̬1:�������� �������ʧ �ұ��߻���
	if(flag_cross_circle == CROSS_CIRCLE_NONE && flag_circle == CIRCLE_NONE && flag_cross == CROSS_NONE && flag_ramp == RAMP_NONE
		&& count_ones_ccd2 > 80 && count_ones_ccd1 <40 && count_ones_ccd2 < 100   //�����������
		&& left_point2 < 15 && right_point2 < 100                                         //�����߽�����
		&& (Now_right_point2 - right_point1) < 30                                         //�ұ���ֱ��
		&& (circle_no_distence >= 39000)                                                 //����Բ��
		&& circle_left_check){

			count_crbegin_flag = count_start;
			flag_circle = CIRCLE_LEFT_BEGIN;
			circle_count = 0;
	}

	//״̬2�����ڶ��ߴ�
	else if(flag_circle == CIRCLE_LEFT_BEGIN) 
	{
//		if(/*��ʶ��Բ���ж�*/ 0){
//			circle_begin_yaw = 0;	
//			flag_circle = CIRCLE_NONE;
//			circle_count = 0;
//		}
		
		circle_in_yaw = yaw;

	//��count_ones_ccd1 ���� count_ones(ccd_data_01,128)��ʡ����
		if(count_ones_ccd1 > 45)
		{
			//��¼begin״̬��yaw�������Ƕ��ж�
			circle_begin_yaw = yaw;		
			flag_circle = CIRCLE_LEFT_IN; 
			distance_saw = circle_begin_distence; //��¼bigin������in״̬��һ�̵�·�� �����жϴ�Բ��СԲ��
			count_crbegin_flag = count_stop;
		}
		
		left_point2 = right_point2 - 53;  //û������һ������ǰ������ߵ����ұ��߼�������
	}
	//״̬3������׼�����뻷���ĵط� 
	else if(flag_circle == CIRCLE_LEFT_IN) {

//		offset_keep_flag = 1;

		circle_yaw_error = yaw_error(circle_begin_yaw,1);
		//ת��һ���ǶȽ�����һ״̬
		
		if(distance_saw <= 45000){offset_keep = 16;offset_keep_flag = 1;}
		else if(distance_saw > 45000)right_point2 = left_point2 + 15;  //�г���Ѳ��
		
		
		if(yaw_error(circle_begin_yaw,1)>60 && yaw_error(circle_begin_yaw,1)<100){ 
			flag_circle = CIRCLE_LEFT_RUNNING;
			offset_keep = 0;
			offset_keep_flag = 0;
			
		}	
		
		

	}
	
/////////////////////////////////////CIRCLE_LEFT_RUNNING/////////////////////////////////////
	
	//״̬6�������� ����д���� ����offsetֵ������get_offset������	
	else if(flag_circle == CIRCLE_LEFT_RUNNING) {
			
		circle_yaw_error = yaw_error(circle_begin_yaw,1);
//		//���2��ת��һ���Ƕȳ���
		
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
	//״̬6�������� ���Ȳ�����
	else if(flag_circle == CIRCLE_LEFT_OUT){//�ùߵ�


		circle_yaw_error = yaw_error(circle_begin_yaw,1);
		target_yaw = circle_begin_yaw;
		offset_keep_flag = 0;
//		if((right_point1 > 100 || right_point2 > 100 )&& circle_left_out_flag == 0){
//			
			//������״̬���̶���ǣ���running״̬��ƽ�����Ч������
//			if(distance_saw <= 45000 && (yaw_error(circle_begin_yaw,1)>260)){offset_keep = 18;offset_keep_flag = 0;}        //С�����
//			else if(distance_saw > 45000 && (yaw_error(circle_begin_yaw,1)>285)){offset_keep = 12;offset_keep_flag = 0;}      //�󻷴��
			// if(circle_left_check)offset_keep = 17;           //С�����
			// else if(circle_left_big_check)offset_keep = 12;       //�󻷴��
				//P32 = 1;
				test0 = 2;
		
	
			if((yaw_error(circle_begin_yaw,1)>330 || yaw_error(circle_begin_yaw,1) < 20) && distance_saw <= 45000 /*&& count_ones_ccd1 > 25 && count_ones_ccd2 > 70*/)circle_left_out_flag = 2;   //�ֿ���С��������

			if((yaw_error(circle_begin_yaw,1)>330 || yaw_error(circle_begin_yaw,1) < 20) && distance_saw > 45000 )circle_left_out_flag = 2;			
//			
		//}

		//�ص���Բ��ƽ���λ�ã�
		//if(circle_left_out_flag == 2){
		
//				P32 = 1;
//				//�����ҵ��ߣ����ô�����
//				offset_keep = 0;
//				offset_keep_flag = 0;

//				circle_count = 0;
//				last_left_point2 = 0;		
//				circle_left_in_flag = 0;	
//				
//				left_point2 = right_point2 - 50;  //����ߵ����ұ��߼������� 
//				test0 = 3;	
//			//�ָ���ͨ�������뿪����
			if((count_ones_ccd2 < 55 && Now_left_point2 > 20 && circle_left_out_flag == 2 && distance_saw <= 45000) ||
				(count_ones_ccd2 < 55 && Now_left_point2 > 20 && count_ones_ccd1 < 40 && circle_left_out_flag == 2 && distance_saw > 45000)){
				P32 = 1;
				//�����ҵ��ߣ����ô�����
				offset_keep = 0;
				offset_keep_flag = 0;
				count_nocri_flag = count_start;
				circle_count = 0;          //���ֱ�־λȫ������
				last_left_point2 = 0;		
				circle_left_in_flag = 0;
				circle_left_out_flag = 0;
				flag_circle = CIRCLE_NONE; //�ָ��ɼ��ĳ�ʼ״̬
				circle_begin_distence = 0;
				distance_saw = 0;
				test0 = 4;
			}
			
		//}
		if( (Now_right_point2-Now_left_point2) > 48 && (Now_right_point2-Now_left_point2)<=54 
				&& offset2 > -22 && offset2 <22 ){
					P32 = 0;
					//�����ҵ��ߣ����ô�����
					offset_keep = 0;
					offset_keep_flag = 0;
					count_nocri_flag = count_start;
					circle_count = 0;          //���ֱ�־λȫ������
					last_left_point2 = 0;		
					circle_left_in_flag = 0;
					circle_left_out_flag = 0;
					flag_circle = CIRCLE_NONE; //�ָ��ɼ��ĳ�ʼ״̬
					circle_begin_distence = 0;
					distance_saw = 0;
					test0 = 4;
				}

	}
	
//////////////////////////////////////////////////////////////��Բ��////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////��Բ��////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////��Բ��////////////////////////////////////////////////////////////
	//״̬1:�������� �ұ�����ʧ ����߻���
	if(flag_cross_circle == CROSS_CIRCLE_NONE && flag_circle == CIRCLE_NONE && flag_cross == CROSS_NONE && flag_ramp == RAMP_NONE
		&& count_ones_ccd2 > 80 && count_ones_ccd1 <40      //�����������
		&& left_point2 > 20 && right_point2 > 100                                         //�����߽�����
		&& (myabs(Now_left_point2 - left_point1)) < 30                                         //�ұ���ֱ��
		/*&& (circle_no_distence >= 39000)*/                                                  //����Բ��
		&& 1){

			count_crbegin_flag = count_start;
			flag_circle = CIRCLE_RIGHT_BEGIN;
			circle_count = 0;

	}

	//״̬2�����ڶ��ߴ�
	else if(flag_circle == CIRCLE_RIGHT_BEGIN) {

//		if(/*��ʶ��Բ���ж�*/ 0){
//			circle_begin_yaw = 0;	
//			flag_circle = CIRCLE_NONE;
//			circle_count = 0;
//		}
		
		circle_in_yaw = yaw;

	//��count_ones_ccd1 ���� count_ones(ccd_data_01,128)��ʡ����
		if(count_ones_ccd1 > 45)
		{

			//��¼begin״̬��yaw�������Ƕ��ж�
			circle_begin_yaw = yaw;		
			flag_circle = CIRCLE_RIGHT_IN; 
			distance_saw = circle_begin_distence; //��¼bigin������in״̬��һ�̵�·�� �����жϴ�Բ��СԲ��
			count_crbegin_flag = count_stop;
			
		}
		
		right_point2 = left_point2 + 53;  //û������һ������ǰ���ұ��ߵ�������߼�������
	}
	//״̬3������׼�����뻷���ĵط� 
	else if(flag_circle == CIRCLE_RIGHT_IN) {
		//������״̬���̶���ǣ���running״̬��ƽ�����Ч������
//		
//		offset_keep_flag = 1;

	    circle_yaw_error = yaw_error(circle_begin_yaw,1);
		//ת��һ���ǶȽ�����һ״̬

		if(yaw_error(circle_begin_yaw,0)>60 && yaw_error(circle_begin_yaw,0)<100){ 
			flag_circle = CIRCLE_RIGHT_RUNNING;
			offset_keep = 0;
			offset_keep_flag = 0;
			
		}	
		
		left_point2 = right_point2 - 53;  //�г���Ѳ��

		
	}
	
/////////////////////////////////////CIRCLE_RIGHT_RUNNING/////////////////////////////////////
	
	//״̬6�������� ����д���� ����offsetֵ������get_offset������	
	else if(flag_circle == CIRCLE_RIGHT_RUNNING) {
			
	    circle_yaw_error = yaw_error(circle_begin_yaw,0);
//		//���2��ת��һ���Ƕȳ���

		if(yaw_error(circle_begin_yaw,0)>220){ 
			
			offset_keep_flag = 0;

			flag_circle = CIRCLE_RIGHT_OUT;
		}
		right_point2 =left_point2  + 58;  //�г���Ѳ��

	}
/////////////////////////////////////CIRCLE_RIGHT_OUT/////////////////////////////////////
	//״̬6�������� ���Ȳ�����
	else if(flag_circle == CIRCLE_RIGHT_OUT){


		circle_yaw_error = yaw_error(circle_begin_yaw,0);
		
		if(left_point2 < 28 && circle_right_out_flag == 0){
			
			//������״̬���̶���ǣ���running״̬��ƽ�����Ч������
			if(distance_saw <= 45000)offset_keep = -16;           //С�����
			else if(distance_saw > 45000)offset_keep = -12;       //�󻷴��
			
			offset_keep_flag = 1;
			circle_right_out_flag = 1 ;  //	ȷ������
		}
		if(circle_right_out_flag == 1)
		{
			P32=1;
//			circle_flag_out_cnt++;
//			if(count_ones_ccd1 > 25 && count_ones_ccd2 > 70 )
//			{
				if((yaw_error(circle_begin_yaw,0)>350 || yaw_error(circle_begin_yaw,0) < 20) && distance_saw <= 45000)circle_right_out_flag = 2;   //�ֿ���С��������
				if((yaw_error(circle_begin_yaw,0)>340 || yaw_error(circle_begin_yaw,0) < 20) && distance_saw > 45000)circle_right_out_flag = 2;
//			}
			
//			if(circle_flag_out_cnt>=30) 	circle_right_out_flag=2;
				
		}
		//�ص���Բ��ƽ���λ�ã�
		else if(circle_right_out_flag == 2)
		{
			P32=0;	
			//�����ҵ��ߣ����ô�����
			offset_keep = 0;
			offset_keep_flag = 0;

			circle_count = 0;
			last_right_point2 = 0;		
			circle_right_in_flag = 0;	
			
			right_point2 = left_point2 + 53;  //�ұ��ߵ�������߼������� 
			
			if(count_ones_ccd1 < 35)circle_count = 1;   //����ʱ���ֻ���״̬����ǰ�������������˸���־λ
			
			//�ָ���ͨ�������뿪����
			if(ccd2_w_count < 53 && right_point2 < 108 && circle_count == 1)
			{
				//�����ҵ��ߣ����ô�����
				offset_keep = 0;
				offset_keep_flag = 0;
				count_nocri_flag = count_start;
				circle_count = 0;          //���ֱ�־λȫ������
				last_right_point2 = 0;		
				circle_right_in_flag = 0;
				circle_right_out_flag = 0;
				flag_circle = CIRCLE_NONE; //�ָ��ɼ��ĳ�ʼ״̬
				circle_begin_distence = 0;
				distance_saw = 0;
				circle_flag_in_cnt=0;
				circle_flag_out_cnt=0;
			}
		}

	}
}
