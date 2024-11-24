
#define KEY1 P17;
#define KEY2 P16;
#define KEY3 P51;
#define KEY4 P50;
//ʹ�ú궨������������µļ��?
#define KEY1_PRESS 1
#define KEY2_PRESS 2
#define KEY3_PRESS 3
#define KEY4_PRESS 4
#define KEY_UNPRESS 0
#define steet_midlle 628

typedef struct {
    uint8 *array;
    size_t size;
} DynamicArray;

typedef struct{
    float target_val;  //Ŀ��ֵ
    float actual_val;  //ʵ��ֵ
    float err;         //ƫ��ֵ
    float err_next;    //��һ��ƫ��ֵ
    float err_last;    //����һ��ƫ��ֵ
    float Kp, Ki, Kd;  //���������֡�΢��ϵ��
    float integral;    //����ֵ
}pid_t;

enum count_flag_g {
	count_start,
	count_stop,
	count_clear,
};


#define DIR P35
extern int	motor_pwm_out;//pwm��ֵ�����?
extern int	steer_pwm_out;//pwm��ֵ�����?
extern int16 dat;//�������ٶ�
extern int key_test;
extern unsigned char TM3_Isr_count;
extern pid_t motor_pid;
extern pid_t steer_pid;
extern pid_t inertia_pid;
extern int key_scan( int mode);
extern int circle_run_point[20];
extern uint16 circle_begin_distence;
extern unsigned char process_flag;//ͣ����־
extern enum count_flag_g count_crbegin_flag;
extern enum count_flag_g count_nocri_flag;
extern enum count_flag_g count_ramp_flag;
extern enum count_flag_g count_cross_flag;
extern enum count_flag_g count_cross_cricle_flag;
extern enum count_flag_g count_turn_pro_flag;
extern enum count_flag_g count_zebra_flag;
extern uint16 circle_no_distence;
extern uint16 ramp_distence;
extern uint16 cross_distence;
extern uint16 cross_cricle_distence;
extern uint16 speed_ramp;
extern uint16 tuen_pro_distence;
extern uint16 zebra_distence;
extern uint8 key_mode;
extern uint8 car_ready_flag;
extern uint8 car_ready_gogogo;
extern float IncPID_realize(pid_t *pid, float temp_val);
extern float PosPID_realize(pid_t *pid, float temp_val);
extern void PID_param_init(pid_t *pid);
extern float pid_realize_a(float actual, float set, float _p, float _d);
extern float filter(float value);
extern void addToArray(DynamicArray *dArray, uint8 value);
extern float calc_ave(int arr[], int size);
extern int countOnes(int arr[], int size);
extern void key_1();
extern int max(int a, int b);
extern int count_ones(uint16 arr[], uint16 n);
extern void car_check();
extern void lcd_show_ccd(uint16 p1[],uint16 p2[]);
extern void display_0();
extern void display_1();
extern void car_init();
extern void KP_get(int8 ccd_offset);
extern float A;
extern float bas_kp;
extern float KP;
extern int test0 ;