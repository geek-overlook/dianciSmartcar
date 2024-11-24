#include "CCD_Driver.h"

uint16 ccd_data_01[128] = {0};
uint16 ccd_data_02[128] = {0};
//�����߿��
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
uint8 circle_count = 0;  //����״̬�ƴ�
uint8 circle_left_in_flag = 0;//�뻷
uint8 circle_right_in_flag = 0;
uint8 circle_left_out_flag = 0;//����
uint8 circle_right_out_flag = 0;
uint8 circle_left_out = 0;//����
uint8 ccd1_circle_left_out = 0;//����


uint16 ccd1_w_count = 0;
uint16 ccd2_w_count = 0;
uint8 ccd_select = 0;
uint8 l_circle_max = 0; //������������
uint8 last_left_point1 = 0;  //��һ�ε�����ߵ�
uint8 last_right_point1 = 0;  //��һ�ε��ұ��ߵ�
uint8 last_left_point2 = 0;  //��һ�ε�����ߵ�
uint8 last_right_point2 = 0;  //��һ�ε��ұ��ߵ�
int8 offset1=0;//���
int8 offset2=0;//���
int8 last_offset=0;//��һ��ƫ��ֵ
int8 offset_inertia=0;//��������������
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


////////////////////////////////////����////////////////////////////////////
//�ֶ�pid
uint32 inertia_kp = 5;
uint32 inertia_kd = 10;
uint32 turn_kp = 9;
uint32 turn_kd = 30;
//�ٶ�
uint16 speed_turn = 840;//700
uint16 speed_cross = 800;//700
uint16 speed_ramp = 600;
uint16 speed_stright = 1150;//1000
int16  limit_motor_pwm = 0;


/////////////////////////////////Ԫ�ؼ���־λ//////////////////////////////
uint8 circle_left_check = 0 ;
uint8 circle_left_big_check = 0 ;
uint8 circle_right_check = 0 ;
uint8 cross_check = 0 ;
uint8 cross_circle_cleck = 0;
uint8 ramp_check = 0 ;
uint8 zebra_flag = 0 ;//��1�ſ�ʼ�������߲�ͣ��

enum flag_circle_e flag_circle=0;
enum flag_cross_circle_e flag_cross_circle=0;
enum flag_ramp_e flag_ramp = 0;
enum flag_cross_e flag_cross = 0;

uint16 ccd_threshold(uint16 ccd_data[])
{
	uint16 i;
	uint16 value1_max, value1_min;
	uint16 CCD_Threshold = 0;

    value1_max = ccd_data[0];      // ��̬��ֵ�㷨����ȡ������Сֵ
    for (i = 5; i < 123; i++) // ���߸�ȥ��5����
    {
        if (value1_max <= ccd_data[i])
            value1_max = ccd_data[i];
    }
    value1_min = ccd_data[0]; // ��Сֵ
    for (i = 5; i < 123; i++)
    {
        if (value1_min >= ccd_data[i])
            value1_min = ccd_data[i];
    }
    CCD_Threshold = (value1_max + value1_min) / 2; // ���������������ȡ����ֵ	
	return CCD_Threshold;
}

//����ccdͻ�����������������
uint16 zebra_stop(uint16 arr[], uint16 n) {
	int8 i;
	uint16 count = 0;
	uint16 prev = arr[0]; // ��¼��һ��Ԫ�ص�ֵ	
	if (n <= 0) {
		return 0; // ���鳤�ȱ������0
	}
	for ( i = 1; i < n; ++i) {
		if (arr[i] != prev) {
			count++; // ���������������1
		}
		prev = arr[i];
	}

	return count;
}



void ccd_two_value(uint16 value1, uint16 value2)//ccd���ݶ�ֵ��  �ڵ�Ϊ0���׵�Ϊ1   value 300����
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
		
		if(temp_num1>value1){ccd_data_01[i]=1;ccd1_w_count++;}//�׵�Ϊ1
		else               {ccd_data_01[i]=0;}//�ڵ�Ϊ0
		
		if(temp_num2>value2){ccd_data_02[i]=1;ccd2_w_count++;}//�׵�Ϊ1
		else               {ccd_data_02[i]=0;}//�ڵ�Ϊ0
	}
}

//��2���㼯�˲�
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

uint16 myabs(int16 number)//����ֵ����
{
	static uint16 abs_number=0;
	 
	if( number>=0)abs_number=number;
	else abs_number=-number;

	return abs_number;

}

uint8 left_point1 =   0;//ccd1��ߵ�
uint8 right_point1= 127;//ccd1�ұߵ�
uint8 mid_point1  =  64;//ccd1�е�

uint8 left_point2 =   0;//ccd2��ߵ�
uint8 right_point2= 127;//ccd2�ұߵ�
uint8 mid_point2  =  64;//ccd2�е�
uint8 last_mid_point2=64;

void get_mid_point1(void)//����ccd1���е�
 {
	static uint8 last_mid_point1=64;
	uint8 l_point1=0;
	uint8 r_point1=0;
	ccd1_circle_left_out = 0;
	for( l_point1=last_mid_point1;l_point1>0;l_point1--)//������ߵ㣬���ϴ��е���Ϊ��ε����ѵ�
	{
		if(ccd_data_01[l_point1-1]==0&&ccd_data_01[l_point1]==0)//�ҵ����������ڵ�
		{
			left_point1=l_point1;
			break;//�ҵ���ߵ��˳�ѭ��
			}
			else if(l_point1==1)//�Ҳ�����ߵ�
			{
				ccd1_l_loss++;
				left_point1=0;//��0��Ϊ��ߵ�
			break;//�˳�ѭ��
			}
	}
	for( r_point1=last_mid_point1;r_point1<127;r_point1++)//�����ұߵ㣬���ϴ��е���Ϊ��ε����ѵ�
	{
		if(ccd_data_01[r_point1+1]==0&&ccd_data_01[r_point1]==0)//�ҵ����������ڵ�
			{
				right_point1=r_point1;
				break;//�ҵ��ұߵ��˳�ѭ��
			}
		else if(r_point1==126)//�Ҳ����ұߵ�
			{
				ccd1_r_loss++;
			right_point1=127;//��127��Ϊ�ұߵ�
			break;//�˳�ѭ��
			}
		}


	// //����ʧ�ܣ�����ڶ�������
	// if((right_point1-left_point1)<=1 && count_ones_ccd1>5 ){
	// 	//���1������ֻ��һ��
	// 	if(zebra_stop(ccd_data_01,128) <= 2){
	// 		//������
	// 		//�����������
	// 		if(ccd_data_01[0]==1){
	// 			left_point1 = 0;
	// 		}
	// 		//���߲�������ߣ��Һڱ�׵ĵ�
	// 		else{
	// 			for(l_point1 = 1 ; l_point1<128 ; l_point1++){
	// 				if(ccd_data_01[l_point1-1]==0 && ccd_data_01[l_point1]==1){
	// 					left_point1 = l_point1;
	// 					break;
	// 				}
	// 			}
	// 		}

	// 		//������
	// 		if(ccd_data_01[127]==1){
	// 			right_point1 = 127;
	// 		}
	// 		//���߲������ұߣ��Һڱ�׵ĵ�
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
	

	
        //    LimitL(left_point1);//��ߵ��޷�
		//    LimitH(right_point1);//�ұߵ��޷�



		//������Բ����ǰҡ,ֻ������ֱ��Բ��
		if((Now_right_point2 -right_point1 >5) && (Now_right_point2 -right_point1 <15) && //�ұ�Ϊֱ��
		((count_ones_ccd1 < 55 && count_ones_ccd2 > 75 )||//ccd2���� ccd1������
		(count_ones_ccd1 > 68 && count_ones_ccd2 < 53 ))&&//ccd1���� ccd2������
		flag_ramp == RAMP_NONE && flag_cross_circle == CROSS_CIRCLE_NONE && flag_circle == CIRCLE_NONE && flag_cross == CROSS_NONE//�����κ�һ��Ԫ��
		){
			left_point1 = right_point1 - 30;
		}

		if((left_point1 - Now_left_point2 >5) && (left_point1 - Now_left_point2 <15) && //���Ϊֱ��
		(count_ones_ccd1 < 55 && count_ones_ccd2 > 75 )||//ccd2���� ccd1������
		(count_ones_ccd1 > 68 && count_ones_ccd2 < 53 )&&//ccd1���� ccd2������
		flag_ramp == RAMP_NONE && flag_cross_circle == CROSS_CIRCLE_NONE && flag_circle == CIRCLE_NONE && flag_cross == CROSS_NONE//�����κ�һ��Ԫ��
		){
			right_point1 = left_point1 + 30;
		}
    
		if(myabs(last_mid_point1- mid_point1)<20)
		{
			mid_point1=(left_point1+right_point1)/2;
		
		}
		
		else//��ʮ��ʱ��ֹ��Ϊ����ڶ��������������������ϴ��е���Ϊ����е�
			
		{
		mid_point1=last_mid_point1;//��ʮ��ʱ��ֹ��Ϊ����ڶ��������������ϴ��е���Ϊ����е�
		
		}
		
		if(myabs(last_mid_point1- mid_point1) == 127)ccd1_circle_left_out = 1;
    last_mid_point1= mid_point1;//������е���Ϊ�´ε��ϴ��е㣬����Ϊ���ѵ�
}

void get_mid_point2(void)//����ccd2���е�
{

	uint8 l_point2=0;
	uint8 r_point2=0;
	static uint8 running_count;
	//������ߵ㣬���ϴ��е���Ϊ��ε����ѵ�
	for( l_point2=last_mid_point2;l_point2>0;l_point2--){
		//�ҵ����������ڵ�
		if(ccd_data_02[l_point2-1]==0&&ccd_data_02[l_point2]==0){
			left_point2=l_point2;
			break;//�ҵ���ߵ��˳�ѭ��
		}
		//�Ҳ�����ߵ�
		else if(l_point2==7){
			left_point2=0;//��0��Ϊ��ߵ�
			ccd2_l_loss++;
			break;//�˳�ѭ��
		}
	}
	//�����ұߵ㣬���ϴ��е���Ϊ��ε����ѵ�
	for( r_point2=last_mid_point2;r_point2<127;r_point2++){
		//�ҵ����������ڵ�
		if(ccd_data_02[r_point2+1]==0&&ccd_data_02[r_point2]==0){
			right_point2=r_point2;
			break;//�ҵ��ұߵ��˳�ѭ��
		}
		//�Ҳ����ұߵ�
		else if(r_point2==120){
			right_point2=120;//��127��Ϊ�ұߵ�
			ccd2_r_loss++ ;
			break;//�˳�ѭ��
		}
	}
	Now_left_point2 = left_point2;
	Now_right_point2 = right_point2;

	//Բ��
	cricle_proccess();

	//������Բ����ǰҡ,ֻ������ֱ��Բ��
	if((Now_right_point2 -right_point1 >5) && (Now_right_point2 -right_point1 <15) && //�ұ�Ϊֱ��
	((count_ones_ccd1 < 55 && count_ones_ccd2 > 75 )||//ccd2���� ccd1������
	(count_ones_ccd1 > 68 && count_ones_ccd2 < 53 ))&&//ccd1���� ccd2������
	flag_ramp == RAMP_NONE && flag_cross_circle == CROSS_CIRCLE_NONE && flag_circle == CIRCLE_NONE && flag_cross == CROSS_NONE//�����κ�һ��Ԫ��
	){
		left_point2 = right_point2 - 51;
	}

	if((left_point1 - Now_left_point2 >5) && (left_point1 - Now_left_point2 <15) && //�ұ�Ϊֱ��
	((count_ones_ccd1 < 55 && count_ones_ccd2 > 75 )||//ccd2���� ccd1������
	(count_ones_ccd1 > 68 && count_ones_ccd2 < 53 ))&&//ccd1���� ccd2������
	flag_ramp == RAMP_NONE && flag_cross_circle == CROSS_CIRCLE_NONE && flag_circle == CIRCLE_NONE && flag_cross == CROSS_NONE//�����κ�һ��Ԫ��
	){
		right_point2 = left_point2 + 51;
	}

	

	last_right_point2 = right_point2;
	LimitL(left_point2);//��ߵ��޷�
	LimitH(right_point2);//�ұߵ��޷�


	if(myabs(last_mid_point2- mid_point2)<20){
		mid_point2=(left_point2+right_point2)/2;
	}
	else{//��ʮ��ʱ��ֹ��Ϊ����ڶ��������������������ϴ��е���Ϊ����е�
		mid_point2=last_mid_point2;//��ʮ��ʱ��ֹ��Ϊ����ڶ��������������ϴ��е���Ϊ����е�
	}
    last_mid_point2= mid_point2;//������е���Ϊ�´ε��ϴ��е㣬����Ϊ���ѵ�
}


//�������
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
		motor_pwm_out = 0;//���߱���
		motor_pid.target_val = (float)0;
	}else if(0){//�ߵ�����
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
	//СԲ������
	else if((flag_circle == CIRCLE_LEFT_RUNNING || flag_circle == CIRCLE_RIGHT_RUNNING) && distance_saw <= 45000){
		motor_pid.target_val = (float)900;
		steer_pid.Kp= turn_kp;				//steer_pid.Kp=KP;
		steer_pid.Kd = turn_kd ;			//steer_pid.Kd = KP *3;
		
	}
	//��Բ������
	else if((flag_circle == CIRCLE_LEFT_IN || flag_circle == CIRCLE_LEFT_RUNNING || flag_circle == CIRCLE_RIGHT_RUNNING) && distance_saw > 45000){
		motor_pid.target_val = (float)920;
		steer_pid.Kp=5;
		steer_pid.Kd = 17;
	}
	//�µ�
	else if(flag_ramp != RAMP_NONE){
		motor_pid.target_val = (float)speed_ramp;
		steer_pid.Kp = inertia_kp;
		steer_pid.Kd = inertia_kd;
		// P32 = 1;
	}
	//ʮ��
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
	//ֱ��
	else if( (Now_right_point2-Now_left_point2) > 48 && (Now_right_point2-Now_left_point2)<=54 
	&& offset2 > -22 && offset2 <22 
	/*&&flag_circle == CIRCLE_NONE*/ && flag_ramp == RAMP_NONE){
		motor_pid.target_val = (float)speed_stright;
		steer_pid.Kp=5;
		steer_pid.Kd = 17;
//		P32 = 0;
	}
	//�������
	else if(offset2 > 25 || offset2 < -25 ){
		motor_pid.target_val = (float)750;
		steer_pid.Kp=KP;
		steer_pid.Kd=KP *3.2;
//		P32 = 0;
	}
	//���
	else{
		motor_pid.target_val = (float)speed_turn;
		steer_pid.Kp=KP;
		steer_pid.Kd=KP *3.2;
//		P32 = 0;
	}



//////////////////////////Ѳ�߷�ʽ�л�//////////////////////////
	if(
		flag_circle != CIRCLE_NONE || flag_cross == CROSS_NONE 
		//���Ǵ�̶��ǶȺ͹ߵ����
		&& flag_ramp != RAMP_NONE && !offset_keep_flag
		&& flag_cross_circle != CROSS_CIRCLE_OUT
		){
		aim_angle_filter = offset2;
		// P32 = 0;
	}
	else 
	{
		//ccd1�������ô�ccd2Ѱ��
		if(right_point1-left_point1 == 0){
			aim_angle_filter = offset2*1.2;
			// P32 = 0;
		}
		else {
			// P32 = 1;
			aim_angle_filter = offset_weight;
		}
	}
	//��Ҫ�ùߵ���״̬��������
	if(flag_ramp == RAMP_IN_MID || flag_cross_circle == CROSS_CIRCLE_OUT || flag_circle == CIRCLE_LEFT_OUT){
		aim_angle_filter = offset_inertia;
	}
	

	//ʮ��
	if(flag_cross != CROSS_NONE  && flag_ramp == RAMP_NONE && !offset_keep_flag){
		//ȫ���ߣ��ߵ�
		if(count_ones(ccd_data_01,128)>89&&count_ones(ccd_data_02,128)>=60){
			flag_cross = CROSS_MID;
			aim_angle_filter = offset_inertia;
			// P32 = 1;
		}
		//ccd1���ߣ�ccd2���ߣ�ѰԶ��
		else if(count_ones(ccd_data_01,128)<40&&count_ones(ccd_data_02,128)>=60 && offset1 < 12 && offset1 > -12){
			flag_cross = CROSS_MID;
			aim_angle_filter = offset_inertia;
			// P32 = 1;
		}
		//ccd1���ߣ�ccd2δ���ߣ�Ѱ����
		else if(count_ones(ccd_data_01,128)>89&&count_ones(ccd_data_02,128)<60){
			
			flag_cross = CROSS_RUNNING;
			aim_angle_filter = offset2;
			get_target_yaw(offset2);
			// P32 = 0;
		}
		// ccd1��ccd2�����ߣ��˳�ʮ��
		else if(count_ones(ccd_data_01,128)<35&&count_ones(ccd_data_02,128)<=55){
			aim_angle_filter = offset2;
			flag_cross = CROSS_NONE;
		}
//		else if(cross_distence > 27300){                     //��μ��� ��֪��Ϊʲô����s���������ʮ���������ʶ���Բ��
//			aim_angle_filter = offset2;
//			flag_cross = CROSS_NONE;                     
//			P32 = 1;
//		}
		else{
			aim_angle_filter = offset_inertia;
		}
	}
	//����λ�ô�̶��Ƕ�
	if(offset_keep_flag){
		aim_angle_filter = offset_keep;
	}
	//�����߼��
	if(zebra_stop(ccd_data_02,128) >= 7 && count_ones_ccd1 >28 && count_ones_ccd1 < 32){//��һ�μ�⵽������
		if(count_ones(ccd_data_02, 128)<7)
			zebra_flag = 1;
			count_zebra_flag = count_start;
	}
	if(zebra_stop(ccd_data_02,128) >= 7 && count_ones_ccd1 >28 && count_ones_ccd1 < 32 && zebra_distence > 50000 && zebra_flag){//��һ�μ�⵽������
		if(count_ones(ccd_data_02, 128)<7){
			process_flag = 0;
		}
	}
	
}



