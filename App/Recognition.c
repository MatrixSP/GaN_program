#include "include.h"

#define ADC_GET_PL adc_once(ADC1_SE11,ADC_8bit)
#define ADC_GET_PM adc_once(ADC0_SE13,ADC_8bit)
#define ADC_GET_PR adc_once(ADC0_SE10,ADC_8bit)
#define ADC_GET_VL adc_once(ADC1_SE10,ADC_8bit)
#define ADC_GET_VR adc_once(ADC0_SE12,ADC_8bit)

int Trace = 0;//赛道元素标志
/*===================================================================
功能：中线卡尔曼滤波
===================================================================*/
#define Q_mid 0.5 
#define R_mid 10
int x_last_mid, x_mid_mid, x_now_mid;
float p_last_mid, p_mid_mid, p_now_mid;
float kg_mid;
/// <summary>
/// 中线卡尔曼滤波
/// </summary>
float Kalman_Filter_mid(float z_measure_mid)
{
	x_mid_mid = x_last_mid;																					//x_last=x(k-1|k-1),x_mid=x(k|k-1)  
	p_mid_mid = p_last_mid + Q_mid;																	//p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
	kg_mid = (float)p_mid_mid / (p_mid_mid + R_mid);//kg为kalman filter，R为噪声
	x_now_mid = x_mid_mid + (float)kg_mid*(z_measure_mid - x_mid_mid);//估计出的最优值
	p_now_mid = (1 - kg_mid)*p_mid_mid;//最优值对应的covariance  
	p_last_mid = p_now_mid;//更新covariance值
	x_last_mid = x_now_mid;//更新系统状态值
	return x_now_mid;
}

//外部定义
extern float piancha;
extern float pre_piancha;
extern int flag_buzz;
/*===================================================================
功能：采集初始化
输入：无
===================================================================*/
void Main_ADC_INIT()
{
	adc_init(ADC1_SE11);
	adc_init(ADC0_SE13);
	adc_init(ADC0_SE12);
	adc_init(ADC1_SE10);
	adc_init(ADC0_SE10);
}

/*===================================================================
功能：加权平均
输入：数据->avg数组，权系数表->weighted数组，数据个数->size
函数测试：6.4日测试无BUG
===================================================================*/
//权重表
# define PianchaFiter_N 5
# define MAXNUM 1     //找出MAXNUM个最大的数
int Weighted_List_cbh[3] = { 1, 4, 1 };
int Weight_Piancha[PianchaFiter_N] = { 16, 4, 4, 1, 1 };
int Weight_Piancha_3[PianchaFiter_N] = { 4, 2, 2, 1, 1 };

/// <summary>
/// int型加权平均，数据->avg数组，权系数表->weighted数组，数据个数->size
/// </summary>
int Weighted_Avg_Int(int avg[], int * weighted, int size)
{
	int index1 = 0;
	int All_Weight = 0;
	long sum = 0;
	for (index1 = 0; index1 < size; index1++)
	{
		All_Weight += weighted[index1];
		sum += (avg[index1] * weighted[index1]);
	}
	return (sum / All_Weight);
}

/// <summary>
/// float型加权平均，数据->avg数组，权系数表->weighted数组，数据个数->size
/// </summary>
float Weighted_Avg_float(float avg[], int * weighted, int size)
{
	int index1 = 0;
	int All_Weight = 0;
	double sum = 0;
	for (index1 = 0; index1 < size; index1++)
	{
		All_Weight += weighted[index1];
		sum += (avg[index1] * weighted[index1]);
	}
	return (sum / All_Weight);
}


/*===================================================================
功能：排列出N个最大值分别对应数组下标0、1、2、……、n-1
输入：数据->arr数组，数据个数->size,最大值个数->maxnum
函数测试：6.4日测试无BUG
===================================================================*/
/// <summary>
/// 排列出N个最大值
/// </summary>
void Sort_MAX_N(int arr[], int size, int maxnum)
{
	int i = 0, k = 0;
	int temp = 0;
	for (k = 0; k<maxnum; k++)
	{
		//降序排列！
		for (i = size - 1; i>k; i--)
		{
			if (arr[i]>arr[i - 1])
			{
				temp = arr[i - 1];
				arr[i - 1] = arr[i];
				arr[i] = temp;
			}
		}
	}
}


/*===================================================================
功能：水平电感归一化
输入：arr_f为归一化前的电感采集数组，arr_o为归一化后的电感采集数组
备注：数组下标表示：0->左电感，1->中电感，2->右电感
函数测试：6.4日测试无BUG
===================================================================*/
int NORMAL_MIN[5] = { 1, 3, 2, 3, 3 }; //归一化用的电感的最小值下标0、1、2、3、4分别表示水平的左、中、右电感值和竖直的左、右电感
int NORMAL_MAX[5] = { 255, 255, 255, 250, 250 }; //归一化用的电感的最大值下标0、1、2、3、4分别表示水平的左、中、右电感值和竖直的左、右电感
/// <summary>
/// 水平电感归一化
/// </summary>
void Normal_P(int * arr_f, int * arr_o)
{
	int i = 0;

	arr_o[0] = (arr_f[0] - NORMAL_MIN[0]) * 100 / (NORMAL_MAX[0] - NORMAL_MIN[0]);
	arr_o[1] = (arr_f[1] - NORMAL_MIN[1]) * 100 / (NORMAL_MAX[1] - NORMAL_MIN[1]);
	arr_o[2] = (arr_f[2] - NORMAL_MIN[2]) * 100 / (NORMAL_MAX[2] - NORMAL_MIN[2]);

	//!+<<<<<<<<<<<<<<<<<<<<限幅>>>>>>>>>>>>>>>>>>>>+//
	for (i = 0; i < 3; i++)
	{
		if (arr_o[i] < 1)
			arr_o[i] = 1;
		if (arr_o[i] > 100)
			arr_o[i] = 100;
	}
}

/*===================================================================
功能：竖直电感归一化
输入：arr_f为归一化前的电感采集数组，arr_o为归一化后的电感采集数组
备注：数组下标表示：0->左电感，1->右电感
函数测试：6.4日测试无BUG
===================================================================*/
/// <summary>
/// 竖直电感归一化
/// </summary>
void Normal_V(int * arr_f, int * arr_o)
{
	int i = 0;


	arr_o[0] = (arr_f[0] - NORMAL_MIN[3]) * 100 / (NORMAL_MAX[3] - NORMAL_MIN[3]);
	arr_o[1] = (arr_f[1] - NORMAL_MIN[4]) * 100 / (NORMAL_MAX[4] - NORMAL_MIN[4]);


	for (i = 0; i < 3; i++)
	{
		if (arr_o[i] < 1)
			arr_o[i] = 1;
		if (arr_o[i] > 100)
			arr_o[i] = 100;
	}
}

/*===================================================================
功能：加权递推平均滤波
输入：num->实时值，queue->滤波队列,weighted->权重表,num_q->滤波个数
===================================================================*/
/// <summary>
/// int型加权递推平均滤波
/// </summary>
int Fiter_int(int num, int * queue, int * weighted, int num_q)
{
	//!+<<<<<<<<<<<<<<<<<<<<更新队列>>>>>>>>>>>>>>>>>>>>+//
	int i = 0;
	for (i = num_q - 1; i > 0; i--)
	{
		queue[i] = queue[i - 1];
	}
	queue[0] = num;

	return Weighted_Avg_Int(queue, weighted, num_q);
}
/*===================================================================
功能：加权递推平均滤波
输入：num->实时值，queue->滤波队列,weighted->权重表,num_q->滤波个数
===================================================================*/
/// <summary>
/// float型加权递推平均滤波
/// </summary>
float Fiter_float(float num, float * queue, int * weighted, int num_q)
{
	//!+<<<<<<<<<<<<<<<<<<<<更新队列>>>>>>>>>>>>>>>>>>>>+//
	int i = 0;
	for (i = num_q - 1; i > 0; i--)
	{
		queue[i] = queue[i - 1];
	}
	queue[0] = num;

	return Weighted_Avg_float(queue, weighted, num_q);
}

/*===================================================================
功能：消抖限幅滤波
输入：num->实时值，queue->滤波队列,N->消抖计数上限,A->限幅幅值
===================================================================*/
# define ED_N 3
# define ED_A 3
int num_count_PL = 0;//消抖滤波计数
int num_count_VL = 0;//消抖滤波计数
int num_count_PM = 0;//消抖滤波计数
int num_count_VR = 0;//消抖滤波计数
int num_count_PR = 0;//消抖滤波计数

int PL_queue_ED[ED_N] = { 0 };
int PM_queue_ED[ED_N] = { 0 };
int PR_queue_ED[ED_N] = { 0 };
int VL_queue_ED[ED_N] = { 0 };
int VR_queue_ED[ED_N] = { 0 };

/*===================================================================
功能：N窗口限幅中值滤波
输入：num->实时值,queue->滤波队列，queue_buff->滤波队列缓存
函数测试：6.4日测试无BUG
===================================================================*/
# define Medium_N 3
int Queue_buff[Medium_N] = { 0 };
int PL_queue_M[Medium_N] = { 0 };
int PM_queue_M[Medium_N] = { 0 };
int PR_queue_M[Medium_N] = { 0 };
int VL_queue_M[Medium_N] = { 0 };
int VR_queue_M[Medium_N] = { 0 };

/// <summary>
/// N窗口限幅中值滤波
/// </summary>
int fiter_Medium(int num, int * queue, int * queue_buff, int size)
{
	int i = 0;


	//!+<<<<<<<<<<<<<<<<<<<<限幅>>>>>>>>>>>>>>>>>>>>+//
	if (num - queue[0] < -4 && num - queue[0]>4)
	{
		num = queue[0];
	}

	//!+<<<<<<<<<<<<<<<<<<<<更新队列>>>>>>>>>>>>>>>>>>>>+//
	for (i = size - 1; i > 0; i--)
	{
		queue[i] = queue[i - 1];
	}
	queue[0] = num;

	//!+<<<<<<<<<<<<<<<<<<<<找中位值>>>>>>>>>>>>>>>>>>>>+//
	for (i = 0; i < size; i++)
		queue_buff[i] = queue[i];

	Sort_MAX_N(queue_buff, size, (1 + size) / 2);

	return queue_buff[(1 + size) / 2 - 1];
}
/*===================================================================
功能：电感采集
输入：
函数测试：6.4日测试无BUG
===================================================================*/
# define N_COLLECT 55//采集次数
# define N_lvbo 5 //电感采集滤波大小
int Vertical_AD_L[N_COLLECT] = { 0 };//采集缓存数组
int Vertical_AD_R[N_COLLECT] = { 0 };//采集缓存数组
int Plane_AD_L[N_COLLECT] = { 0 };//采集缓存数组
int Plane_AD_M[N_COLLECT] = { 0 };//采集缓存数组
int Plane_AD_R[N_COLLECT] = { 0 };//采集缓存数组
int Plane_AD[3] = { 0 };//下标0、1、2分别表示水平的左、中、右电感值
int PL_queue[N_lvbo] = { 0 };
int PM_queue[N_lvbo] = { 0 };
int PR_queue[N_lvbo] = { 0 };
int VL_queue[N_lvbo] = { 0 };
int VR_queue[N_lvbo] = { 0 };
int Weight_AD[N_lvbo] = { 1, 1, 1, 1, 2 };
float Plane_AD_N_New[3] = { 0 };//下标0、1、2分别表示水平的左、中、右电感值
int Plane_AD_N[3] = { 0 };//下标0、1、2分别表示水平的左、中、右电感值
int Vertical_AD[2] = { 0 };//下标0、1分别表示竖直的左、右电感值
int Vertical_AD_N[2] = { 0 };//下标0、1分别表示竖直的左、右电感值
float Vertical_AD_N_New[2] = { 0 };//下标0、1分别表示竖直的左、右电感值
int Distance_MID[5] = { 0 };//下标0、1、2、3、4分别表示水平的左、中、右电感值和竖直的左、右电感

void AD_Collect()
{
	int i = 0, j = 0;
	float rate_buff = 0;

	//!+<<<<<<<<<<<<<<<<<<<<ADC采集>>>>>>>>>>>>>>>>>>>>+//
	for (i = 0; i < N_COLLECT; i++)
	{
		Plane_AD_L[i] = ADC_GET_PL;
		Plane_AD_M[i] = ADC_GET_PM;
		Plane_AD_R[i] = ADC_GET_PR;
		Vertical_AD_L[i] = ADC_GET_VL;
		Vertical_AD_R[i] = ADC_GET_VR;
	}

	//!+<<<<<<<<<<<<<<<<<<<<算出波形峰值>>>>>>>>>>>>>>>>>>>>+//
	//数组第一个元素就是最大值了
	Sort_MAX_N(Plane_AD_L, N_COLLECT, MAXNUM);
	Sort_MAX_N(Plane_AD_M, N_COLLECT, MAXNUM);
	Sort_MAX_N(Plane_AD_R, N_COLLECT, MAXNUM);
	Sort_MAX_N(Vertical_AD_L, N_COLLECT, MAXNUM);
	Sort_MAX_N(Vertical_AD_R, N_COLLECT, MAXNUM);

	/*
	//!+<<<<<<<<<<<<<<<<<<<<取均值>>>>>>>>>>>>>>>>>>>>+//
	Plane_AD[0] = Weighted_Avg_Int(Plane_AD_L, Weighted_List_Collect, MAXNUM);
	Plane_AD[1] = Weighted_Avg_Int(Plane_AD_M, Weighted_List_Collect, MAXNUM);
	Plane_AD[2] = Weighted_Avg_Int(Plane_AD_R, Weighted_List_Collect, MAXNUM);
	Vertical_AD[0] = Weighted_Avg_Int(Vertical_AD_L, Weighted_List_Collect, MAXNUM);
	Vertical_AD[1] = Weighted_Avg_Int(Vertical_AD_R, Weighted_List_Collect, MAXNUM);
	*/
	//!+<<<<<<<<<<<<<<<<<<<<取出最大值>>>>>>>>>>>>>>>>>>>>+//
	Plane_AD[0] = Plane_AD_L[0];
	Plane_AD[1] = Plane_AD_M[0];
	Plane_AD[2] = Plane_AD_R[0];
	Vertical_AD[0] = Vertical_AD_L[0];
	Vertical_AD[1] = Vertical_AD_R[0];


	//////!+<<<<<<<<<<<<<<<<<<<<平均滤波，大小5>>>>>>>>>>>>>>>>>>>>+//
	//Fiter_int(Plane_AD[0], PL_queue,Weight_AD,N_lvbo);
	//Fiter_int(Plane_AD[1], PM_queue, Weight_AD, N_lvbo);
	//Fiter_int(Plane_AD[2], PR_queue, Weight_AD, N_lvbo);
	//Fiter_int(Vertical_AD[0], VL_queue, Weight_AD, N_lvbo);
	//Fiter_int(Vertical_AD[1], VR_queue, Weight_AD, N_lvbo);
	/*
	//!+<<<<<<<<<<<<<<<<<<<<消抖限幅，大小2，限幅3>>>>>>>>>>>>>>>>>>>>+//
	Plane_AD[0] = fiter_EDA(Plane_AD[0],PL_queue_ED,num_count_PL,2,3);
	Plane_AD[1] = fiter_EDA(Plane_AD[1], PM_queue_ED, num_count_PM, 2, 3);
	Plane_AD[2] = fiter_EDA(Plane_AD[2], PR_queue_ED, num_count_PR, 2, 3);
	Vertical_AD[0] = fiter_EDA(Vertical_AD[0], VL_queue_ED, num_count_VL, 2, 3);
	Vertical_AD[1] = fiter_EDA(Vertical_AD[1], VR_queue_ED, num_count_VR, 2, 3);
	*/

	//!+<<<<<<<<<<<<<<<<<<<<五窗口中值滤波>>>>>>>>>>>>>>>>>>>>+//
	Plane_AD[0] = fiter_Medium(Plane_AD[0], PL_queue_M, Queue_buff, Medium_N);
	Plane_AD[1] = fiter_Medium(Plane_AD[1], PM_queue_M, Queue_buff, Medium_N);
	Plane_AD[2] = fiter_Medium(Plane_AD[2], PR_queue_M, Queue_buff, Medium_N);
	Vertical_AD[0] = fiter_Medium(Vertical_AD[0], VL_queue_M, Queue_buff, Medium_N);
	Vertical_AD[1] = fiter_Medium(Vertical_AD[1], VR_queue_M, Queue_buff, Medium_N);

	//!+<<<<<<<<<<<<<<<<<<<<归一化>>>>>>>>>>>>>>>>>>>>+//
	Normal_P(Plane_AD, Plane_AD_N);
	Normal_V(Vertical_AD, Vertical_AD_N);
}

/*===================================================================
功能：计算方向环偏差
输入：
===================================================================*/
float PianchaFiter[PianchaFiter_N] = { 0 };
float error_lr_N = 0;//左右电感的偏差（归一化后的）
float error_Vlr_N = 0;
float average_distance = 0;
float sum_plane = 0;
float I_error = 0;
float R_error = 0;
float piancha_test = 0;
extern float Dir_Diff;
extern int16 GYRO_z;
extern float DIRC_P, DIRC_D, DIRC_I;
int flag_turn_left = 0;
int flag_turn_right = 0;
int flag_const = 0;
extern float yuanhuan_distance;
/// <summary>
/// Trace=4进圆环偏差算法
/// </summary>
int flag_Trace5 = 0;//稳定5模式标志位
float rate_e3 = 4;
float rate_e4_dj = 1.3;//1.3
float rate_e4 = 9;
float rate_e6 = 1;
# define Dajia_50 140
# define Dajia_100 280
int NUM_O = 4;//圆环总个数
int count_O = -1;//进圆环个数计数
int radium_save[12] = { 50, 100, 50, 100, 50, 100, 50, 100, 50, 100, 50, 100 };
int distance_test = 0;//距离
int flag_count_S = 0;//测距标志位
void Error_4(int radium, int flag_dajiao)//偏差拟合算法函数
{
	float error_buff = 0;

	float Start_buff = 0;
	float rate_buff = 0;
	float K = 0;
	float B = 0;

	K = (0 - 46) / 50;
	B = 46 - 50 * K;
	Start_buff = K*radium + B;

	K = (0 - 5) / 50;
	B = 5 - 50 * K;
	rate_buff = K*radium + B;
	

	if (Trace == 4 || Trace == 5)//测距标志
	{
		flag_count_S = 1;
	}
	else
	{
		flag_count_S = 0;
	}
	if (flag_turn_right)
	{
		error_lr_N = (float)(Plane_AD[1] - Plane_AD[2]);
		error_Vlr_N = -1 * (float)(Vertical_AD_N[0] - Vertical_AD_N[1]);
		error_buff = error_lr_N;
		piancha_test = rate_e4 * error_buff;

		if (distance_test >= 0 && distance_test < 35)//开始打角前
		{
			if (flag_turn_right)
				error_lr_N = (float)(Plane_AD[0] - Plane_AD[1]);//L-M
			else if (flag_turn_left)
				error_lr_N = (float)(Plane_AD[1] - Plane_AD[2]);//M-R
			sum_plane = (float)(Weighted_Avg_Int(Plane_AD, Weighted_List_cbh, 3));
			piancha_test = error_lr_N * rate_e3;//error_lr_N * 100 / sum_plane;
		}
	}
	else if (flag_turn_left)
	{
		error_lr_N = (float)(Plane_AD[0] - Plane_AD[1]);
		error_Vlr_N = (float)(Vertical_AD_N[1] - Vertical_AD_N[0]);
		error_buff = error_lr_N;
		piancha_test = rate_e4 * error_buff;
		if (distance_test >= 0 && distance_test < 35)//开始打角前
		{
			if (flag_turn_right)
				error_lr_N = (float)(Plane_AD[0] - Plane_AD[1]);//L-M
			else if (flag_turn_left)
				error_lr_N = (float)(Plane_AD[1] - Plane_AD[2]);//M-R
			sum_plane = (float)(Weighted_Avg_Int(Plane_AD, Weighted_List_cbh, 3));
			piancha_test = error_lr_N * rate_e3;//error_lr_N * 100 / sum_plane;
		}
	}

	
}
int flag_CheckT6 = 0;//开始检测6模式标志位
/*===================================================================
功能：4模式固定打角
输入：radium:圆环半径
===================================================================*/
extern float DIRC_P;
int DAJIAO_START = 70;
int DAJIAO_END = 140;
int DAJIAO_Clear = 1000;
//小环参数
# define DAJIA_SMALL 600//18~20:730 8~10:600 
# define DJ_START_S 55
# define DJ_END_S 140
# define DAJIA_BIG 400//18~20:545  8~10:400
# define DJ_START_B 150
# define DJ_END_B 320

float Speed_O = 0;//入环速度

void ConstError_4(int radium)
{
	int dajiao = 0;
	float K = 0;
	float B = 0;
	//线性赋值部分
	//打角赋值
	K = (DAJIA_BIG - DAJIA_SMALL) / 50;
	B = DAJIA_SMALL - K * 50;
	dajiao = K*radium+B;
	//速度打角赋值
	K = (19 - 9) / 9;
	B = 9 - K * 9;
	dajiao = K*Speed_O + B;

	//开始值赋值
	K = (DJ_START_B - DJ_START_S) / 50;
	B = DJ_START_S - K * 50;
	DAJIAO_START = K*radium + B;
	//结束值赋值
	K = (DJ_END_B - DJ_END_S) / 50;
	B = DJ_END_S - K * 50;
	DAJIAO_END = K*radium + B;



	if (flag_count_S == 1)
	{

		//!+<<<<<<<<<<<<<<<<<<<<固定打角控制部分>>>>>>>>>>>>>>>>>>>>+//
		if (distance_test >= 0 && distance_test < DAJIAO_START)//开始打角前
		{
			if (flag_turn_right)
				error_lr_N = (float)(Plane_AD[0] - Plane_AD[1]);//L-M
			else if (flag_turn_left)
				error_lr_N = (float)(Plane_AD[1] - Plane_AD[2]);//M-R
			sum_plane = (float)(Weighted_Avg_Int(Plane_AD, Weighted_List_cbh, 3));
			piancha = error_lr_N * rate_e3;//error_lr_N * 100 / sum_plane;
		}
		if (distance_test >= DAJIAO_START && distance_test <= DAJIAO_END)//打角部分
		{
			if (flag_turn_left)
				piancha = dajiao / DIRC_P;
			else if (flag_turn_right)
				piancha = -dajiao / DIRC_P;
		}
		if (distance_test > DAJIAO_END && distance_test < DAJIAO_Clear)//打角结束
		{

		}
		if (distance_test >= DAJIAO_Clear || flag_count_S == 0)//打角清零
		{
			distance_test = 0;
		}
	}
}
void Error_Calculation(void)
{
	//简单调系数
	//if (radium_save[count_O] == 100)
	//{
	//	rate_e3 = 2;
	//	rate_e4 = 6;
	//	rate_e6 = 1;
	//}
	//if (radium_save[count_O] == 50)
	//{
	//	rate_e3 = 6;
	//	rate_e4 = 9;
	//	rate_e6 = 1;
	//}
	//!+<<<<<<<<<<<<<<<<<<<<分模式偏差公式>>>>>>>>>>>>>>>>>>>>+//
	switch (Trace)
	{
		//!+<<<<<<<<<<<<<<<<<<<<圆环部分偏差计算>>>>>>>>>>>>>>>>>>>>+//
	case 3://圆环标识
	{
			   if (flag_turn_right)
				   error_lr_N = (float)(Plane_AD[0] - Plane_AD[1]);//L-M
			   else if (flag_turn_left)
				   error_lr_N = (float)(Plane_AD[1] - Plane_AD[2]);//M-R
			   sum_plane = (float)(Weighted_Avg_Int(Plane_AD, Weighted_List_cbh, 3));
			   piancha = error_lr_N * rate_e3;//error_lr_N * 100 / sum_plane;
			   break;
	}
	case 4://进圆环状态
	{
			   //if (radium_save[count_O]>75)//环大不打角
			   // Error_4(radium_save[count_O], 0);
			   //else
			   Error_4(radium_save[count_O], 0);
			   piancha = piancha_test;
			   break;
	}
	case 5://圆环内状态
	{
			   /*
			   error_lr_N = (float)(Plane_AD_N[0] - Plane_AD_N[2]);//L-R
			   sum_plane = (float)(Weighted_Avg_Int(Plane_AD_N, Weighted_List_cbh, 3));
			   piancha = error_lr_N * 100 / sum_plane;
			   */

			   //if (distance_test >= 650)
			   //{
				   flag_CheckT6 = 1;
			   //}
			   //else
			   //{
				  // flag_CheckT6 = 0;
			   //}

			   error_lr_N = (float)(Plane_AD_N[0] - Plane_AD_N[2]);//L-R
			   sum_plane = (float)(Weighted_Avg_Int(Plane_AD_N, Weighted_List_cbh, 3));
			   piancha = error_lr_N * 100 / sum_plane;

			   break;
	}
	case 6://出圆环标志
	{
			   error_lr_N = (float)(Plane_AD_N[0] - Plane_AD_N[2]);//L-R
			   sum_plane = (float)(Weighted_Avg_Int(Plane_AD_N, Weighted_List_cbh, 3));
			   piancha = error_lr_N * 100 / sum_plane;
			   piancha = rate_e6*piancha;
			   break;
	}
	case 8://出圆环状态
	{
			   //if (flag_turn_left)
			   //	error_lr_N = (float)(Plane_AD[1] - Plane_AD[2]);//M-R
			   //else if (flag_turn_right)
			   //	error_lr_N = (float)(Plane_AD[0] - Plane_AD[1]);//L-M
			   //sum_plane = (float)(Weighted_Avg_Int(Plane_AD, Weighted_List_cbh, 3));
			   //piancha = error_lr_N * 9;//error_lr_N * 100 / sum_plane;
			   //break;

			   error_lr_N = (float)(Plane_AD_N[0] - Plane_AD_N[2]);//L-R
			   sum_plane = (float)(Weighted_Avg_Int(Plane_AD_N, Weighted_List_cbh, 3));
			   piancha = error_lr_N * 100 / sum_plane;
			   piancha = rate_e6*piancha;
			   break;
	}
		//case 8:
		//{
		//		if (flag_turn_left)
		//		  	error_lr_N = (float)(Plane_AD[1] - Plane_AD[2]);//M-R
		//		else if (flag_turn_right)
		//		  	error_lr_N = (float)(Plane_AD[0] - Plane_AD[1]);//L-M
		//		sum_plane = (float)(Weighted_Avg_Int(Plane_AD, Weighted_List_cbh, 3));
		//		piancha = error_lr_N * 9;//error_lr_N * 100 / sum_plane;
		//		break;
		//}
		//!+<<<<<<<<<<<<<<<<<<<<普通部分偏差计算>>>>>>>>>>>>>>>>>>>>+//
	default:
	{
			   error_lr_N = (float)(Plane_AD_N[0] - Plane_AD_N[2]);//L-R
			   sum_plane = (float)(Weighted_Avg_Int(Plane_AD_N, Weighted_List_cbh, 3));
			   piancha = error_lr_N * 100 / sum_plane;

			   break;
	}
	}
	//!+<<<<<<<<<<<<<<<<<<<<圆环固定打角部分>>>>>>>>>>>>>>>>>>>>+//
	if (flag_const == 1)
	{
		ConstError_4(radium_save[count_O]);//550
	}
	//!+<<<<<<<<<<<<<<<<<<<<丢线处理部分>>>>>>>>>>>>>>>>>>>>+//
	Lose_Pro();

	////!+<<<<<<<<<<<<<<<<<<<<瞎鸡巴滤波>>>>>>>>>>>>>>>>>>>>+//
	//if (Trace == 3 || Trace == 6 || Trace == 7 || Trace == 8 || Trace == 5)
	//	piancha = Kalman_Filter_mid(piancha);
	//if (Trace == 3 || Trace == 6 || Trace == 7 || Trace == 8 || Trace == 5)
	//	piancha = Fiter_float(piancha, PianchaFiter, Weight_Piancha_3, PianchaFiter_N);
	//else
		//!+<<<<<<<<<<<<<<<<<<<<偏差加权递推平均滤波>>>>>>>>>>>>>>>>>>>>+//
		piancha = Fiter_float(piancha, PianchaFiter, Weight_Piancha, PianchaFiter_N);

	//GYRO_z = 0.07*Get_Gyro(5, 'Z') - 15;
}

/*===================================================================
功能：赛道特征变量归一化(使实际情况的放大倍数在计算时设置回调试时候的状态)
输入：
===================================================================*/
//电感最大值：PL->240 PR->240 VL->240 VR->240 PM由赛道中线三电感值一致设定
//以下设定值都是归一化后的值
# define Plane_M_Set 41//中电感在一般情况下的普通最大值（赛道中线上）
# define Sum_Inductance_Set 90//赛道中线上三电感的和的设定值
# define PL_set 23//左电感在赛道中线上的设定值
# define PM_set 41//中电感在赛道中线上的设定值
# define PR_set 23//右电感在赛道中线上的设定值

float PL_fact = 0;//左电感在赛道中线上的实际值
float PM_fact = 0;//中电感在赛道中线上的实际值
float PR_fact = 0;//右电感在赛道中线上的实际值
float Sum_Inductance_fact = 0;//赛道中线上三电感的和的实际值
int AD_Feature[5] = { 0 };//AD特征数组
/// <summary>
/// ！目前还没正式写该函数！
/// </summary>
void Feature_Normal(void)
{
	AD_Feature[0] = Plane_AD_N[0];
	AD_Feature[1] = Plane_AD_N[1];
	AD_Feature[2] = Plane_AD_N[2];
	AD_Feature[3] = Vertical_AD_N[0];
	AD_Feature[4] = Vertical_AD_N[1];
}
/*===================================================================
功能：0\1\12\2\3的分类决策树,主要是为了区分出3模式
输出：返回模式3：3，模式2：2，模式0：0
===================================================================*/
int CTree_3(void)
{
	int x1 = AD_Feature[0] + AD_Feature[2];//左右水平电感和
	int x2 = abs(AD_Feature[3] - AD_Feature[4]);//左右竖直电感和
	int x3 = AD_Feature[3] + AD_Feature[4];

	if (x1 >= 75)//水平电感和至少要大于等于87 75
	{
		if (x1 >= 97)//十字处水平电感最大值 85
			return 3;
		else//2 3 12模式混在一起的部分
		{
			//x2<=50区分十字 x2 >= 15区分坡道
			if (x2<=50 && x2 >= 15 && x3 > 82)//竖直电感和很大一定是圆环
				return 3;
			else
				return 0;
		}
	}
	else//0 2模式混在一起的部分
	{
		return 0;
	}


}
/*===================================================================
功能：赛道标志检测
输入：
===================================================================*/
int r_flag = 0;//弯道标志,编号：0
int x_flag = 0;//十字标志,编号：2
int ss_flag = 0;//小S标志,编号：11
int p_flag = 0;//坡道标志,编号：12
int o_flag = 0;//圆环标志,编号：3
int o_clear_flag = 0;//圆环清楚标志
int o_flag_enter = 0;//进圆环标志,编号：4
int o_flag_in = 0;//圆环内标志,编号：5
int o_flag_out = 0;//出圆环标志,编号：6
int o_flag_outing = 0;//出圆环时标志,编号：7
int o_flag_out_end = 0;//出圆环结束标志,编号：8
int sum_inductance = 0;//5电感的和
int sum_inductance_F = 0;
/// <summary>
/// 赛道标志检测
/// </summary>
void CheckTrace(int flag_index)
{
	int i = 0;
	int num_buff = 0;
	Feature_Normal();

	sum_inductance_F = AD_Feature[0] + AD_Feature[2];//左右水平电感的和;

	switch (flag_index)
	{
	case 1://直道标志,决策树+SVM
	{
			   num_buff = CTree_3();
			   if (num_buff == 3)
			   {
				   o_flag++;
			   }
			   else if (num_buff == 0)
			   {
				   r_flag++;
			   }
			   break;
	}
	case 4://入圆环标志
	{

			   if (abs(AD_Feature[3] - AD_Feature[4]) <= 12)//Ctree_4() == 4)
			   {
				   o_flag_enter++;
			   }

			   num_buff = CTree_3();
			   if (num_buff == 0)
			   {
				   o_clear_flag++;
			   }

			   break;
	}
	case 5://圆环内标志
	{
			   if (sum_inductance_F < 60 && (AD_Feature[3] + AD_Feature[4]) >= 29)//1.59
				   o_flag_in++;
			   break;
	}
	case 6:	//圆环标志
	{
				if (sum_inductance_F >= 60 && (AD_Feature[3] + AD_Feature[4]) >= 72)//sum_inductance_F >= 60)
				{
					o_flag_out++;
				}
				break;
	}
	case 7:
	{
			  if ((AD_Feature[0] + AD_Feature[1]+ AD_Feature[2]) > 89)
			  {
				  o_flag_outing++;
			  }
			  break;
	}
	case 8:
	{
			  if (sum_inductance_F < 58)
			  {
				  o_flag_out_end++;
			  }
			  break;
	}
	default:
	{
			   r_flag++;
			   break;
	}
	}
}
/*===================================================================
功能：赛道标志清0
输入：
===================================================================*/
/// <summary>
/// 赛道标志清0
/// </summary>
void Flag_Clear()
{
	r_flag = 0;//弯道标志,编号：2
	o_flag = 0;//圆环标志,编号：3
	o_flag_enter = 0;//进圆环标志,编号：4
	o_flag_in = 0;//圆环内标志,编号：5
	o_flag_out = 0;//出圆环标志,编号：6
	o_flag_out_end = 0;
	o_flag_outing = 0;
	x_flag = 0;
	ss_flag = 0;
	p_flag = 0;
	o_clear_flag = 0;
}




/*===================================================================
功能：模式识别
输入：
===================================================================*/
extern float  nAngleControlOut, nSpeedControlOut, nDirControlOut;
extern int flag_stop;
extern float nSpeed;
//Trace表：
//0->弯道 1->直道 2-> 十字 11-> 小S 3->前面有圆环 4->入圆环 5->圆环内 6->出圆环状态 7->已出圆环
void ModeRecognition(void)
{
	if (o_flag == 10)
	{
		Trace = 3;

		//!+<<<<<<<<<<<<<<<<<<<<检测左右转>>>>>>>>>>>>>>>>>>>>+//
		if (Vertical_AD[0] > Vertical_AD[1])
		{
			flag_turn_right = 0;
			flag_turn_left = 1;
		}
		else if (Vertical_AD[1] > Vertical_AD[0])
		{
			flag_turn_left = 0;
			flag_turn_right = 1;
		}
		Flag_Clear();
	}
	if (Trace == 3 && o_clear_flag == 12)//圆环标志清除
	{
		Trace = 0;
		Flag_Clear();
	}
	if (o_flag_enter == 6)//进圆环判断
	{
		count_O++;//进了一次圆环
		if (count_O >= NUM_O)
			count_O = 0;
		Speed_O = nSpeed;
		Trace = 4;
		yuanhuan_distance = 0;
		Flag_Clear();
	}
	if (o_flag_in == 6)//圆环内判断
	{
		Trace = 5;
		Flag_Clear();
	}
	if (x_flag == 8)//十字判断
	{
		Trace = 2;
		Flag_Clear();
	}
	//else if (p_flag == 7)
	//{
	//	Trace = 12;
	//	Flag_Clear();
	//}
	//else if (ss_flag == 7)
	//{
	//	Trace = 11;
	//	Flag_Clear();
	//}
	if (o_flag_out == 6)//出圆环标志判断
	{
		Trace = 6;
		Flag_Clear();
	}
	if (o_flag_outing == 6)
	{
		Trace = 7;
		Flag_Clear();
	}
	if (o_flag_out_end == 6)//出圆环结束判断
	{
		Trace = 8;
		yuanhuan_distance = 0;
		flag_CheckT6 = 0;
		Flag_Clear();
	}
	if (r_flag == 10)//弯道判断
	{
		Trace = 0;
		Flag_Clear();
	}

	//!+<<<<<<<<<<<<<<<<<<<<圆环识别部分>>>>>>>>>>>>>>>>>>>>+//
	if (Trace == 3)
	{
		CheckTrace(4);
		//CheckTrace(1);
	}
	if (Trace == 4)
	{
		CheckTrace(5);
	}
	if (Trace == 5)
	{
		if (flag_CheckT6 == 1)
		{
			CheckTrace(6);
		}
	}
	if (Trace == 6)
	{
		flag_Trace5 = 0;
		CheckTrace(8);
	}
	if (Trace == 7)
	{
		CheckTrace(8);
	}
	if (Trace == 8)
	{
		flag_turn_left = 0;
		flag_turn_right = 0;
		CheckTrace(0);
	}
	//!+<<<<<<<<<<<<<<<<<<<<其它识别部分>>>>>>>>>>>>>>>>>>>>+//
	if (Trace < 3 || Trace >8)
	{
		CheckTrace(1);
	}

	////!+<<<<<<<<<<<<<<<<<<<<保护程序>>>>>>>>>>>>>>>>>>>>+//
	//if ((Plane_AD[0] <= 20) && (Plane_AD[1] <= 20) && (Plane_AD[2] <= 20))
	//	flag_stop = 1;
	//else
	//	flag_stop = 0;
}
/*===================================================================
功能：丢线处理程序
输入：
===================================================================*/
#define PM_MIN 12//丢线时中电感最大值

float PM__lose_set = 60;
float rate_pc = 4.2;
int flag_lose = 0;//丢线标志位
int flag_lose_left = 0;//左丢线标志
int flag_lose_right = 0;//右丢线标志

float piancha_lose = 0;//丢线偏差

void Lose_Pro()
{
	if (AD_Feature[1] < PM_MIN)
		flag_lose = 1;
	else
		flag_lose = 0;

	if (flag_lose == 1)
	{
		if (Plane_AD_N[0] <17 && Plane_AD_N[2] < 17)//全丢线
		{
			if (flag_lose_right == 1)
				piancha = 800;
			else if (flag_lose_left == 1)
				piancha = -800;
			return;
		}
		//单边丢线
		if (Plane_AD_N[0] > Plane_AD_N[2])//如果右丢线
		{
			piancha_lose = (PM__lose_set - Plane_AD_N[0])*rate_pc;
			piancha = piancha_lose;
			flag_lose_left = 0;
			flag_lose_right = 1;
		}
		else if (Plane_AD_N[0] <= Plane_AD_N[2])
		{
			piancha_lose = (Plane_AD_N[0] - PM__lose_set)*rate_pc;
			piancha = piancha_lose;
			flag_lose_left = 1;
			flag_lose_right = 0;
		}
	}
	else
	{
		flag_lose_left = 0;
		flag_lose_right = 0;
	}
}