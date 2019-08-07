#include "camera.h"
#include "common.h"
#include "include.h"


int Pic[Height][Wide];


/*===================================================================
功能：跳变阈值计算，近处阈值大，远处阈值小
===================================================================*/
unsigned char yuzhi[40];
void yuzhi_cal(void)           //实验室起步50，tiyu
{
	int i;
	for (i = 0; i <= 39; i++)
	{
		yuzhi[i] = 50 + 0.5*i;
	}
}


uint8 qipao = 0;
unsigned char xieru_shizi_flag = 0;
int Midline = 0;
extern float piancha;
int b;
int b1;
int b2[5];
int b3[5];
int space;
int zhuanxiang = 0;
int with[40];
int qianzhan = 0;
int error_piancha[6];
int error_change[3];
int piancha_result;



/***************前后车变量声明**************/
int zhongdianzhangai_location = 2;   //0没有障碍  1左  2右


float steer_zhi_kp = 1; // 1;
float steer_zhi_kd = 3; //   3
float steer_xiaos_kp = 0.5;
float steer_xiaos_kd = 0.6;
float steer_wan_kp_left = 0.8;  // 0.8
float steer_wan_kd_left = 2;    //    2
float steer_wan_kp_right = 0.8; //   0.8
float steer_wan_kd_right = 2;   //  2              //1.5;
float steer_enter_huandao_kp = 0.8;
float steer_enter_huandao_kd = 2;
float steer_exit_huandao_kp = 0.8;
float steer_exit_huandao_kd = 2;
float steer_mid_huandao_kp = 1.8;
float steer_mid_huandao_kd = 2;
float chasu_rate = 0.26;    //0.3

extern int16 speed_right, speed_left, speed_set, speed_set_temp, speed_set_to_left, speed_set_to_right, speed, second_slow_speed, first_slow_speed;



extern int error_distance[4];
extern uint16 avg_distance;
extern int16 rightline[40];

uint8 imgbuff[CAMERA_R_H][CAMERA_W]; //定义存储接收图像的数组
uint8 imgpro[CAMERA_R_H][CAMERA_W];  //二值化处理图像数组
uint8 V_Cnt = 0;
uint8 img_flag = 1; //图像状态
uint16 VS = 0;
uint16 HS = 0;
int16 mid_old = 160;
int16 midline[40] = { 0 };
uint16 steer_out;
int i, j;
int count;
uint8 break_right, break_left;
int16 leftline[40];
int16 rightline[40];
uint8 leftline_flag, rightline_flag;
uint8 xiaos_flag, shizi_flag, dawan_flag;
uint8 shizi_flag_zhi;
uint8 start_find;
unsigned char find_line_right, find_line_left = 0;
unsigned char sao_zuo, sao_you = 1;
unsigned char shizi_flag, shizi_flag_zhi = 0;
unsigned char white = 0;
unsigned char start_find, allwhite = 0;
float slope1, slope2, slope;
unsigned char xierushizi_flag = 0;
unsigned char tiaobian = 0;
unsigned char shizi_flag_xie = 0;
unsigned char shizi_star = 0;
unsigned char find = 0;
unsigned char break_left_shizi, break_right_shizi = 0;
uint8 shizi_star_right, shizi_star_left;
uint8 xiaos_flag = 0;
int zhongxian_front[4] = { 0 };
int zhongxian_back[4] = { 0 };
uint8 zhidao_flag, wandao_flag = 0;
int error_steer[6] = { 0 };
uint8 count_ruwan, count_chuwan = 0;
uint8 ruwan_flag, chuwan_flag = 0;
int zhongxian_front_ave, zhongxian_back_ave = 0;
int tubian = 0;

long time_now = 0;
uint8 weight[20] = { 20, 19, 18, 17, 16, 15, 14, 13, 12, 11,
10, 9, 8, 7, 6, 5, 4, 3, 2, 1 };
uint8 width[30] = { 44,51,58,67,74,83,91,98,105,113,119,126,135,143,150,157,163,170,176,183,189,196,202,209,215,221,227,234,240,246 };
/*uint8 width[40] = {
  41,42,43,44,45,46,48,51,53,56,59,61,64,69,71,75,78,82,86,90,
  93,97,99,103,107,111,113,116,119,121,124,127,130,133,136,139,140,143,145,148 };*/
uint8 kuandu[40];

int panduan_flag = 0;
float slope_xierushizi;
int time;
/***************环岛判断变量声明**************/
int tiaobian_left = 0;
int tiaobian_right = 0;
uint8 ruhuandao_flag = 0;
uint8 chuhuandao_flag = 1;
uint8  white_flag = 0;
int up_dot_zuo = 0;
int up_dot_you = 0;
uint8 huandao_up_dot_zuo = 0;
uint8 huandao_up_dot_you = 0;
int  tiaobian_middle = 0;
int huandao_tiaobian = 0;
int16 up_dot_line[11];
int count_black1, count_black2;   //1代表左边，2代表右边
int edge_length1, edge_length2;
int kuandu_cha1, kuandu_cha2;
int16 up_lie[11];
int huandao_width1, huandao_width2;
int chuhuan_lie, chuhuan_hang;
int up_lie_right, up_lie_left;
int distance1, distance3;      //入环岛测量距离，出环岛测量距离
int huan_buxian_lie, huan_buxian_hang;
float slope_huan, slope_chuhuan, slope_chuhuan1;
float  slope_tiaobian_chu1, slope_tiaobian_chu2;
int count_white, huandao_diu_count;
int huandao_find_right, huandao_find_left;
int huandao_count = 1;
extern int huandao_set[8], huandao_chaoche_set[8], huandao_size_set[8];   //0右边，1左边
int huandao[8];
int huandao_number = 7;
int huandao_houche = 0;
bool ruhuandao_ready, ruhuandao_may;
float slope_chuhuan_zuo, slope_chuhuan_you, slope_ruhuan_zuo, slope_ruhuan_you;
int chuhuan_diu_count, chuhuan_find_left, chuhuan_find_right, ruhuan_diu_count, ruhuan_find_left, ruhuan_find_right;
int zuoruhuan_diu_count, zuoruhuan_diu_find, youruhuan_diu_count, youruhuan_diu_find;
float slope_ruhuan_diu_zuo, slope_ruhuan_diu_you;
float slope_you_shizi, slope_huanzuo_judge, slope_huanyou_judge;
int16 huan_rightline[40], huan_leftline[40];
long huan_temp;
int huan_zhongxian_front;
int shizi_judge_mid;
int huan_judge_bottom1, huan_judge_bottom2, huan_judge_top1, huan_judge_top2;
int right_jump, left_jump, jump_width;
uint8 huandao_size, change_num;
float slope_zuo_judge, slope_you_judge;
bool huan_you_flag, first_diu_you_flag, huan_zuo_flag, first_diu_zuo_flag;
int ruhuan_break_right, ruhuan_break_left;
float slope_huanyou_old, slope_huanzuo_old;
int zuo_ruhuan_start, you_ruhuan_start;
bool chuhuandao_ceju_flag = 0;
bool huandao_stop_flag;
bool huanjiasu_flag = 0, qiandengdai_flag = 0;
/***************障碍变量声明**************/
int16 zhangai_line_zuo[CAMERA_R_H], zhangai_line_you[CAMERA_R_H], zhangai_line_you1[CAMERA_R_H];
int16 zhangai_start, zhangai_break, zhangai_start_zuo, zhangai_start_you;
uint8 zhangaifind_flag, zhangaifind_you_flag, zhangaifind_zuo_flag;
int zhangai_max, zhangai_min, zhangai_middle;
uint8 zhangai_left_flag, zhangai_right_flag, zhangai_flag;
int OA_replace_flag = 0, OB_replace_flag = 0;
int OA_distance = 0;
uint16 zhangai_lilun_middle;
int8 zhangai_back_flag;
int64 OA_distance2 = 0, OB_distance = 0;
int zhangai_count;
float b_right, b_left;
float slope_shizi_left, slope_shizi_right, slope_zhangai_qian;
int zhangai_final;
float slope_zhangai_you, slope_zhangai_zuo;
float za_slope;
int bizhang_flag;
/***************超车变量声明**************/
extern bool cruise_mode_on;
int16 chaoche_line_zuo[CAMERA_R_H], chaoche_line_you[CAMERA_R_H];
int16 chaoche_start_zuo, chaoche_start_you, chaoche_start, chaoche_break;
uint8 chaoche_left_flag, chaoche_right_flag, chaoche_flag, chaochefind_flag, chaochefind_zuo_flag;
bool jieshouchaoche_flag;
int zhidao_chaoche_count = 1;
extern int zhidao_chaoche_num;
uint8 chaoche_ready_flag;
float cc_slope;
int chaoche_distance;
uint8 send_num;
uint8 send_show_number;
int chaoche_distance;
int safe_distance, front_distance;
uint8 turn_flag, Left_turn_flag, tingche_chaoche_flag;
int dajiao_distance, chaoche_stop_distance, shizi_hui_distance;
int midline_drift;
int hui_flag;
bool chaochesudu_flag;
int zhubei_chaoche_flag, shizi_hui_flag;
/***************十字超车变量声明**************/
bool shizi_chaoche_flag, shizi_ready_flag, shizi_tingche_flag, houche_ready_flag;
int shizi_chaoche_distance, zhongxian_stop, shizi_front_distance;
/***************通讯**************/
uint8 relen;
uint8 buff[DATA_PACKET];
uint8 start_flag = 0;
int send_count;
int receive_count;
extern void nrf_tx_mode();
extern void nrf_rx_mode();
/***************十字声明**************/
int shizi_count = 0;
int shizi_distance = 0;
uint8 bizhang_zhidao_flag;
int shizi_tiaobian, shizi_kuandu;
int qinxie_zhi, xrshizi_diu_count, xrshizi_find_right;
float slope_xrszdiu, slope_shizi_judge, slope_zuo_shizi;
uint8 shizi_buxian_flag, shizizhi_judge;
float slpoe_xieshizi_left, slpoe_xieshizi_right;
int shizi_start_left, shizi_start_right;
//调试
int start_refind, shizi_restart, shizi_start_mid;
int xieshizi_tiaobian, xieshizi_jump_width;
int xieshizi_zuodiu_count, xieshizi_youdiu_count, xieshizi_find_left, xieshizi_find_right, xieshizi_break_right, xieshizi_break_left, xieshizi_rebreak_left, xieshizi_you_restart, xieshizi_rebreak_right;
float slpoe_xieshizi_leftback, slpoe_xieshizi_leftfront, slpoe_xieshizi_rightback, slpoe_xieshizi_rightfront;
int xieshizi_left_tiaobian, xieshizi_right_tiaobian, zhishizi_left_tiaobian, zhishizi_right_tiaobian;
float slope_xieshizi_you, slope_xieshizi_zuo;
bool qianshizi_flag1, qianshizi_flag2;
/***************赛道判断声明**************/
float slope_front_left, slope_front_right, slope_back_left, slope_back_right;
float cha_xlv_front, cha_xlv_back, break_start;
int avg_front = 0, curve = 0, avg_front_shiji = 0;
int valid_line, fangcha, fangcha_cha, fangcha_old = 0;
int16 left_cha[40], right_cha[40], mid_cha[40];
int chaoche_zhidao_flag;
int zhangai_distance = 0;
int  allwhite1 = 0;
int kuandu_sum, kuandu_avg;  //识别坡道  赛道宽度
bool zhenzhidao_flag, podao_zhidao_flag;
int aobian_left, aobian_right;
/***************起跑线声明**************/
int tubian_count = 0;
bool qipaoxian_flag = 1, stop_flag = 0, qipaoxian_flag_fa;
int stop_distance = 0;
bool zhongdianjiasu_flag = 0;
extern int speedset_st;
uint8 steer_qian = 0;
bool keep_front_steer_flag;
/***************发车变量声明**************/
extern int fache_flag; //0为正常发车 1为起跑超车发车
int fache_distance;
bool fache_finish_flag = 0;
int fache_state = 0;   //发车过程中的状态变化标志，是中转量  
/***************出赛道变量声明**************/
bool chusaidao_flag;
int black;
/***************坡道变量声明**************/
int podao_distance = 0;
int podao_leave_distance = 10000;
bool podao_ready_flag;
int podao_count;
bool podao_flag = 0;
bool shangpo_flag, qianpodao_flag;
extern  bool tuoluopodao_flag;
/***************偏差变量声明**************/
int error_I = 0;
extern float DIRC_D, DIRC_P, DIRC_I;
/***************判断车辆丢失变量声明**************/
int diushi_count = 0, diushi_time = 0;
int qipaoxian_count = 0;
/*********************************************
！！最小二乘法公式！！
*********************************************/
int SumX = 0;
int SumY = 0;
long x_2 = 0;
long x_y = 0;
float AverageX;
float AverageY;
float AverageX_2;
float AverageX_Y;
int xielv_cal;
int AvaliableLines;

int Least_Squares(int16 *array, int16 Number, int16 start)
{
	AvaliableLines = Number;
	SumX = 0;
	SumY = 0;
	x_2 = 0;
	x_y = 0;

	for (i = start; i >= start - Number + 1; i--) //数组溢出
	{
		SumX = SumX + i;
		SumY = SumY + (*(array + i));
		x_2 = x_2 + i * i;
		x_y = x_y + i * (*(array + i));
	}

	AverageX = (float)SumX / AvaliableLines;
	AverageY = (float)SumY / AvaliableLines;
	AverageX_2 = (float)x_2 / AvaliableLines;
	AverageX_Y = (float)x_y / AvaliableLines;

	xielv_cal = (AverageX_Y - AverageX * AverageY) * 10 /
		(AverageX_2 - AverageX * AverageX);

	return xielv_cal;
}

/*uint8 sz[CAMERA_R_H] = {
  30, 31, 32, 34, 36, 38, 41, 44, 47, 51, 55, 59, 64,
  69, 74, 80, 86, 92, 98, 104, 110, 116, 122, 128, 134, 140,
  146, 152, 158, 164, 170, 176, 182, 188, 194, 200, 206, 212, 218,
  224 //,230
};*/
/*uint8 sz[CAMERA_R_H] = {
  10,12,14,16,18,20,22,24,26,28,
  30,32,34,36,38,40,42,44,46,48,
  50,52,54,56,58,60,62,64,66,68,
  70,72,74,76,78,80,82,84,86,88
};*/
uint8 sz[CAMERA_R_H] = {
  10,12,14,16,18,20,22,24,26,28,
  30,32,34,36,38,40,42,44,46,48,
  50,52,54,56,58,60,62,64,66,68
};

void camera(void)
{
	//LP_image();
	MedianFilter();
	/*************各种标志清零********************/
	sao_zuo = 1;
	sao_you = 1;
	leftline_flag = 0;
	rightline_flag = 0;
	break_right = 39;
	break_left = 39;
	/*************确定基础寻线开始的点***************/
	if (mid_old <= 1 + Interval)
	{
		sao_zuo = 0;
		mid_old = 1 + Interval;
	}
	else if (mid_old >= CAMERA_W - Interval - 2)
	{
		sao_you = 0;
		mid_old = CAMERA_W - Interval - 2;
	}

	/*************开始扫描最底行左线*******************/
	if (sao_zuo == 1 && qipaoxian_flag == 0)
	{
		for (i = mid_old; i >= Interval + 1; i--)
		{
			if (imgbuff[Last_line][i] - imgbuff[Last_line][i - Interval] > Jump_threhold && imgbuff[Last_line][i] - imgbuff[Last_line][i - Interval - 1] > Jump_threhold) //  多加了一个扫描点
			{
				leftline[Last_line] = i - Interval;
				leftline_flag = 1;
				break;
			}
			else
			{
				leftline_flag = 0;
			}
		}
	}
	/*************开始扫描最底行右线*******************/
	if (sao_you == 1 && qipaoxian_flag == 0)
	{
		for (i = mid_old; i <= CAMERA_W - Interval - 2; i++)
		{
			if (imgbuff[Last_line][i] - imgbuff[Last_line][i + Interval] > Jump_threhold&&imgbuff[Last_line][i] - imgbuff[Last_line][i + Interval + 1] > Jump_threhold) //  多加了一个扫描点
			{
				rightline[Last_line] = i + Interval;
				rightline_flag = 1;
				break;
			}
			else
			{
				rightline_flag = 0;
			}
		}
	}
	/*************起跑线扫描最底行左线*******************/
	if (sao_zuo == 1 && qipaoxian_flag == 1)
	{
		for (i = 0; i <= mid_old - Interval - 2; i++)
		{
			if (imgbuff[Last_line][i + Interval] - imgbuff[Last_line][i] > Jump_threhold &&imgbuff[Last_line][i + Interval + 1] - imgbuff[Last_line][i + 1] > Jump_threhold &&imgbuff[Last_line][i + Interval + 2] - imgbuff[Last_line][i + 2] > Jump_threhold)//  多加了一个扫描点
			{
				leftline[Last_line] = i + Interval;
				leftline_flag = 1;
				break;
			}
			else
			{
				leftline_flag = 0;
			}
		}
	}
	/*************起跑线扫描最底行右线*******************/
	if (sao_you == 1 && qipaoxian_flag == 1)
	{
		for (i = CAMERA_W - 1; i >= mid_old + Interval + 2; i--)
		{
			if (imgbuff[Last_line][i - Interval] - imgbuff[Last_line][i] > Jump_threhold && imgbuff[Last_line][i - Interval - 1] - imgbuff[Last_line][i - 1] > Jump_threhold &&imgbuff[Last_line][i - Interval - 2] - imgbuff[Last_line][i - 2] > Jump_threhold)//  多加了一个扫描点
			{
				rightline[Last_line] = i - Interval;
				rightline_flag = 1;
				break;
			}
			else
			{
				rightline_flag = 0;
			}
		}
	}
	/*************最底行补线程序*****************/
	if (leftline_flag == 1 && rightline_flag == 0)
	{
		rightline[Last_line] = leftline[Last_line] + 246; // + 148;
		if (rightline[Last_line] < CAMERA_W)
		{
			rightline[Last_line] = CAMERA_W;
		}
	}

	if (leftline_flag == 0 && rightline_flag == 1)
	{
		leftline[Last_line] = rightline[Last_line] - 246;// - 148
		if (leftline[Last_line] > 0)
		{
			leftline[Last_line] = 0;
		}
	}

	midline[Last_line] = (rightline[Last_line] + leftline[Last_line]) / 2;
	mid_old = midline[Last_line];

	/***************基本跟踪算法*********************/
	//左线跟踪
	if (leftline_flag == 1 && qipaoxian_flag == 0)
	{
		for (int i = CAMERA_R_H - 2; i >= 0; i--) // 40
		{
			for (j = Tracking_NUM; j >= 0; j--) ///    18
			{
				if (leftline[i + 1] + j - Interval - Tracking_displacement >= 0 && leftline[i + 1] + j - Tracking_displacement < CAMERA_W) //   9
				{
					if (imgbuff[i][leftline[i + 1] + j - Tracking_displacement] - imgbuff[i][leftline[i + 1] + j - Interval - Tracking_displacement] > (Jump_threhold - 0.3 * (39 - i))) //  0.3
					{
						leftline[i] = leftline[i + 1] + j - Interval - Tracking_displacement;
						find_line_left = 1;
						break_left = 0;
						break;
					}
					else
					{
						find_line_left = 0;
					}
				}
			}
			if (find_line_left == 0)
			{
				break_left = i + 1;
				break;
			}
		}
	}

	//右线跟踪
	if (rightline_flag == 1 && qipaoxian_flag == 0)
	{
		for (int i = CAMERA_R_H - 2; i >= 0; i--)
		{
			for (j = 0; j <= Tracking_NUM; j++) //  18
			{
				if (rightline[i + 1] + j - Tracking_displacement >= 0 && rightline[i + 1] + j - Tracking_displacement + Interval < CAMERA_W)
				{
					if (imgbuff[i][rightline[i + 1] + j - Tracking_displacement] - imgbuff[i][rightline[i + 1] + j - Tracking_displacement + Interval] > (Jump_threhold - 0.3 * (39 - i)))       // gai imgbuff[i][rightline[i + 1] + j - Tracking_displacement -Interval] -imgbuff[i][rightline[i + 1] + j - Tracking_displacement] >(Jump_threhold - 0.3 * (39 - i))
					{
						rightline[i] = rightline[i + 1] + j - Tracking_displacement + Interval;
						find_line_right = 1;
						break_right = 0;
						break;
					}
					else
					{
						find_line_right = 0;
					}
				}
			}
			if (find_line_right == 0)
			{
				break_right = i + 1;
				break;
			}
		}
	}
	//左线起跑线跟踪
	if (leftline_flag == 1 && qipaoxian_flag == 1)
	{
		for (int i = CAMERA_R_H - 2; i >= 0; i--) // 40
		{
			for (j = 0; j <= Tracking_NUM; j++) ///    18
			{
				if (leftline[i + 1] + j + Interval - Tracking_displacement < CAMERA_W &&leftline[i + 1] + j - Tracking_displacement >= 0) //   9
				{
					if (imgbuff[i][leftline[i + 1] + j + Interval - Tracking_displacement] - imgbuff[i][leftline[i + 1] + j - Tracking_displacement] > (Jump_threhold - 0.3 * (39 - i))) //  0.3
					{
						leftline[i] = leftline[i + 1] + j + Interval - Tracking_displacement;
						find_line_left = 1;
						break_left = 0;
						break;
					}
					else
					{
						find_line_left = 0;
					}
				}
			}
			if (find_line_left == 0)
			{
				break_left = i + 1;
				break;
			}
		}
	}

	//右线起跑线跟踪
	if (rightline_flag == 1 && qipaoxian_flag == 1)
	{
		for (int i = CAMERA_R_H - 2; i >= 0; i--)
		{
			for (j = Tracking_NUM; j >= 0; j--) //  18
			{
				if (rightline[i + 1] + j - Interval - Tracking_displacement >= 0 && rightline[i + 1] + j - Tracking_displacement < CAMERA_W)
				{
					if (imgbuff[i][rightline[i + 1] + j - Interval - Tracking_displacement] - imgbuff[i][rightline[i + 1] + j - Tracking_displacement] > Jump_threhold - 0.3 * (39 - i))
					{
						rightline[i] = rightline[i + 1] + j - Interval - Tracking_displacement;
						find_line_right = 1;
						break_right = 0;
						break;
					}
					else
					{
						find_line_right = 0;
					}
				}
			}
			if (find_line_right == 0)
			{
				break_right = i + 1;
				break;
			}
		}
	}

	/******************************************************
	处理起跑线
	******************************************************/
	tubian_count = 0;
	if (bizhang_zhidao_flag == 1 && qipaoxian_flag == 0)    //前车识别起跑线停车 
	{
		for (int i = 17; i >= 15; i--)
		{
			for (j = zhongxian_front[0] - width[i] / 2; j <= zhongxian_front[0] + width[i] / 2; j++)// zhongxian_front[0]
			{
				if ((imgbuff[i][j] - imgbuff[i][j + 3] > Jump_threhold) && (imgbuff[i][j] - imgbuff[i][j + 4] > Jump_threhold) && imgbuff[i][j] > 150 && imgbuff[i][j + 3] < 150)//白到黑
				{
					tubian_count++;
				}
				if (tubian_count > 18)
				{
					qipaoxian_flag = 1;
					break;
				}
			}
			if (qipaoxian_flag == 1)
			{
				break;
			}
			else
			{
				tubian_count = 0;
			}
		}
	}

	//****直入十字判断*****//
	shizi_flag_zhi = 0;
	if ((break_left >= 35 || break_right >= 35) && abs(break_right - break_left) <= 10) ////可能出现十字的情况，两边同时无法跟踪，且差的绝对值不大   //5  //加入入环岛标志位=0
	{
		if (break_left <= break_right) //确定开始搜索全白图像的起点
		{
			start_find = break_left;
		}
		else
		{
			start_find = break_right;
		}
		if (zhongxian_front[1] < 0 || zhongxian_front[1]>199)    //shizi_flag_zhi!=0
		{
			if (break_right != 39 && break_left != 39)
			{
				shizi_judge_mid = (rightline[CAMERA_R_H - 1] + leftline[CAMERA_R_H - 1]) / 2;
			}
			else if (break_right == 39 && break_left != 39)
			{
				shizi_judge_mid = leftline[CAMERA_R_H - 1] + 100;
			}
			else if (break_left == 39 && break_right != 39)
			{
				shizi_judge_mid = rightline[CAMERA_R_H - 1] - 100;
			}
			else
			{
				shizi_judge_mid = mid_old;
			}
		}
		else if (zhongxian_front[1] >= 0 && zhongxian_front[1] <= 199)     //shizi_flag_zhi!=0
		{
			shizi_judge_mid = zhongxian_front[1];
		}
		count = 0;
		if (shizi_count == 0)
		{
			if (shizi_judge_mid >= 0 && shizi_judge_mid <= 199)
			{
				for (i = 39; i >= 3; i--)
				{
					if (imgbuff[i][shizi_judge_mid] - imgbuff[i - 3][shizi_judge_mid] > Jump_threhold&&imgbuff[i][shizi_judge_mid] > ThreadHold&&imgbuff[i - 3][shizi_judge_mid] < ThreadHold)
					{
						shizi_tiaobian = i - 3;      ////// //////!!!!!!!!!!!!!!!!!!!!!!
						break;
					}
					else
					{
						shizi_tiaobian = 0;
					}
				}
			}
			else
			{
				shizi_tiaobian = 0;
			}
		}
		for (j = start_find - 1; j >= start_find - 4; j--)
		{
			allwhite = 0;
			for (i = 0; i <= CAMERA_W - 1; i++)
			{
				if (imgbuff[j][i] > 150) //直接对当前行进行检测，判断是否为全白
				{
					allwhite++;
				}
			}
			if (allwhite >= CAMERA_W - 5)
			{
				allwhite = 0;
				count++; //判断当白点数过多时，则认为前方为十字，开始从前往后补线
			}
			if (count >= 2)
			{
				if (shizi_count == 0)
				{
					shizi_kuandu = 39 - shizi_tiaobian;
					if (abs(shizi_kuandu) > 25)
					{
						shizi_flag_zhi = 1;
						shizi_count = 1;
					}
				}
				if (shizi_count == 2)
				{
					shizi_count = 3;
				}
				break;
			}
		}
	}

	if (shizi_count == 1)
	{
		//shizi_distance = (speed_left + speed_right) / 2 + shizi_distance;
		if (shizi_distance >= 1200)
		{
			shizi_count = 2;
			shizi_distance = 0;
			qianshizi_flag1 = 0;  ///////清掉前十字标志位
		}
	}
	if (shizi_count == 3)
	{
		//shizi_distance = (speed_left + speed_right) / 2 + shizi_distance;
		if (shizi_distance >= 1100)
		{
			shizi_count = 0;
			shizi_distance = 0;
			qianshizi_flag2 = 0;///////清掉前十字标志位
		}
	}

	/**************寻找十字重新开始行*************/
	if ((shizi_count == 1 || shizi_count == 3) && (zhongxian_front[1]<Interval + 1 || zhongxian_front[1]>CAMERA_W - Interval - 2))    //    (shizi_count==1||shizi_count==3)
	{
		if (break_right != 39 && break_left != 39)
		{
			shizi_start_mid = (rightline[CAMERA_R_H - 1] + leftline[CAMERA_R_H - 1]) / 2;
		}
		else if (break_right == 39 && break_left != 39)
		{
			shizi_start_mid = leftline[CAMERA_R_H - 1] + 100;
		}
		else if (break_left == 39 && break_right != 39)
		{
			shizi_start_mid = rightline[CAMERA_R_H - 1] - 100;
		}
		else
		{
			shizi_start_mid = mid_old;
		}
	}
	else if ((shizi_count == 1 || shizi_count == 3) && zhongxian_front[1] >= Interval + 1 && zhongxian_front[1] <= CAMERA_W - Interval - 2)     //    (shizi_count==1||shizi_count==3)
	{
		shizi_start_mid = zhongxian_front[1];
	}

	/*********************直入十字补线程序**************************/
	if ((shizi_count == 1 || shizi_count == 3) && shizi_start_mid >= Interval + 1 && shizi_start_mid <= CAMERA_W - Interval - 2) //如果十字标志位等于一，那么就继续向前寻找可以跟踪的黑线     shizi_count==1||shizi_count==3
	{
		shizi_star_left = 39;
		count = 0;
		for (i = start_find - 3; i >= 0; i--)
		{
			for (j = shizi_start_mid; j >= Interval + 1; j--)
			{
				if (imgbuff[i][j] - imgbuff[i][j - Interval] > Jump_threhold&&imgbuff[i][j] > 150 && imgbuff[i][j - Interval]<150 && imgbuff[i][j] - imgbuff[i][j - Interval - 1]>Jump_threhold) /////////   跳变大小？？？？？？？？   /////////
				{
					count++;
					leftline[i] = j - Interval;        //leftline[i]=j-Interval;
					break;
				}
			}
			if (count == 3)
			{
				count = 0;
				shizi_star_left = i;
				break;
			}
		}

		shizi_star_right = 39;
		count = 0;
		for (i = start_find - 3; i >= 0; i--)
		{
			for (j = shizi_start_mid; j <= CAMERA_W - Interval - 2; j++)
			{
				if (imgbuff[i][j] - imgbuff[i][j + Interval] > Jump_threhold&&imgbuff[i][j] > 150 && imgbuff[i][j + Interval]<150 && imgbuff[i][j] - imgbuff[i][j + Interval + 1]>Jump_threhold) /////////   跳变大小？？？？？？？？   /////////
				{
					count++;
					rightline[i] = j + Interval;
					break;
				}
			}
			if (count == 3)
			{
				count = 0;
				shizi_star_right = i;
				break;
			}
		}
		if (abs(shizi_star_right - shizi_star_left) <= 7)
		{
			if (shizi_star_right < shizi_star_left) //当再次找到开始的黑线位置时，将开始位置靠前的赋值给实际开始位置
			{
				shizi_start_left = shizi_star_right;
				shizi_start_right = shizi_star_right;
			}
			else
			{
				shizi_start_right = shizi_star_left;
				shizi_start_left = shizi_star_left;
			}
		}
		else
		{
			if (shizi_star_right < shizi_star_left) //当再次找到开始的黑线位置时，将开始位置靠前的赋值给实际开始位置
			{
				shizi_start_left = shizi_star_left;
				shizi_start_right = 0;
			}
			else
			{
				shizi_start_right = shizi_star_right;
				shizi_start_left = 0;
			}
		}
		if (shizi_start_right != 0 || shizi_start_left != 0)
		{
			/**************重新开始的十字黑线跟踪程序*****************/
			if (shizi_start_left != 0)
			{
				for (j = shizi_start_mid; j >= Interval; j--) //开始寻找十字开始的基准点            /////////////!!!!!!!!!!!!!!
				{
					if (imgbuff[shizi_start_left][j] - imgbuff[shizi_start_left][j - Interval] > Jump_threhold&&imgbuff[shizi_start_left][j] > 150 && imgbuff[shizi_start_left][j - Interval] < 150)
					{
						leftline[shizi_start_left] = j - Interval;
						break;
					}
				}
				break_left_shizi = 39;
				for (i = shizi_start_left - 1; i >= 0; i--)   // shizi_star
				{
					for (j = Tracking_NUM; j >= 0; j--)
					{
						if (leftline[i + 1] + j - Tracking_displacement > 0 && leftline[i + 1] + j + Interval - Tracking_displacement < CAMERA_W)
						{
							if (imgbuff[i][leftline[i + 1] + j + Interval - Tracking_displacement] - imgbuff[i][leftline[i + 1] + j - Tracking_displacement] > Jump_threhold&&imgbuff[i][leftline[i + 1] + j + Interval - Tracking_displacement] > 150 && imgbuff[i][leftline[i + 1] + j - Tracking_displacement] < 150)
							{
								leftline[i] = leftline[i + 1] + j - Tracking_displacement;
								find_line_left = 1;
								break_left_shizi = 0;
								break;
							}
							else
							{
								find_line_left = 0;
							}
						}
					}
					if (find_line_left == 0)
					{
						break_left_shizi = i + 1;
						break;
					}
				}
			}
			if (shizi_start_right != 0)
			{
				for (j = shizi_start_mid; j <= CAMERA_W - Interval - 1; j++)
				{
					if (imgbuff[shizi_start_right][j] - imgbuff[shizi_start_right][j + Interval] > Jump_threhold&&imgbuff[shizi_start_right][j] > 150 && imgbuff[shizi_start_right][j + Interval] < 150)
					{
						rightline[shizi_start_right] = j + Interval;
						break;
					}
				}
				break_right_shizi = 39;
				for (i = shizi_start_right - 1; i >= 0; i--)  // shizi_star
				{
					for (j = 0; j <= Tracking_NUM; j++)
					{
						if (rightline[i + 1] + j - Interval - Tracking_displacement > 0 && rightline[i + 1] + j - Tracking_displacement < CAMERA_W)
						{
							if (imgbuff[i][rightline[i + 1] + j - Interval - Tracking_displacement] - imgbuff[i][rightline[i + 1] + j - Tracking_displacement] > Jump_threhold&&imgbuff[i][rightline[i + 1] + j - Interval - Tracking_displacement] > 150 && imgbuff[i][rightline[i + 1] + j - Tracking_displacement] < 150)
							{
								rightline[i] = rightline[i + 1] + j - Tracking_displacement;
								find_line_right = 1;
								break_right_shizi = 0;
								break;
							}
							else
							{
								find_line_right = 0;
							}
						}
					}
					if (find_line_right == 0)
					{
						break_right_shizi = i + 1;
						break;
					}
				}
			}

			// 开始十字向前补线程序！！
			if (break_left_shizi != 0 && shizi_start_left != 0)
			{
				if (shizi_start_right == 0 && shizi_start_left > 30)
				{
					slope = (float)(leftline[shizi_start_left - 3] + leftline[shizi_start_left - 2] - leftline[shizi_start_left - 1] - leftline[shizi_start_left]) / 4;
					for (i = shizi_start_left - 3; i >= 0; i--)
					{
						leftline[i] = leftline[shizi_start_left - 3] + slope*(shizi_start_left - i - 3);
					}
				}
				else
				{
					slope = (float)(leftline[break_left_shizi + 1] + leftline[break_left_shizi] - leftline[break_left_shizi + 3] - leftline[break_left_shizi + 2]) / 4;
					for (i = break_left_shizi + 3; i >= 0; i--)
					{
						leftline[i] = leftline[break_left_shizi + 3] + slope*(break_left_shizi - i + 3);
					}
				}
			}
			if (break_right_shizi != 0 && shizi_start_right != 0)
			{
				if (shizi_start_left == 0 && shizi_start_right > 30)
				{
					slope = (float)(rightline[shizi_start_right - 3] + rightline[shizi_start_right - 2] - rightline[shizi_start_right - 1] - rightline[shizi_start_right]) / 4;
					for (i = shizi_start_right - 3; i >= 0; i--)
					{
						rightline[i] = rightline[shizi_start_right - 3] + slope*(shizi_start_right - i - 3);
					}
				}
				else
				{
					slope = (float)(rightline[break_right_shizi + 1] + rightline[break_right_shizi] - rightline[break_right_shizi + 3] - rightline[break_right_shizi + 2]) / 4;
					for (i = break_right_shizi + 3; i >= 0; i--)
					{
						rightline[i] = rightline[break_right_shizi + 3] + slope*(break_right_shizi - i + 3);
					}
				}
			}

			//***************最小二乘法**************//
			if (shizi_start_right - 2 > break_right_shizi)
			{
				AvaliableLines = 3;
				SumX = 0;
				SumY = 0;
				x_2 = 0;
				x_y = 0;

				for (i = shizi_start_right; i >= shizi_start_right - 2; i--)
				{
					SumX = SumX + (i);
					SumY = SumY + rightline[i];
					x_2 = x_2 + (i)*(i);
					x_y = x_y + i*rightline[i];
				}

				AverageX = (float)SumX*1.0 / AvaliableLines;
				AverageY = (float)SumY*1.0 / AvaliableLines;
				AverageX_2 = (float)x_2*1.0 / AvaliableLines;
				AverageX_Y = (float)x_y*1.0 / AvaliableLines;

				b_right = (AverageX_Y - AverageX*AverageY) * 100 / (AverageX_2 - AverageX*AverageX);
				slope_shizi_right = (float)b_right / 100;
				if (slope_shizi_right <= 0.5 || slope_shizi_right >= 2.6)
				{
					slope_shizi_right = 2.0;
				}
				for (i = shizi_start_right; i <= CAMERA_R_H - 1; i++)
				{
					rightline[i] = rightline[shizi_start_right] - slope_shizi_right * (shizi_start_right - i);
				}
			}

			if (shizi_start_left - 2 > break_left_shizi)
			{
				AvaliableLines = 3;
				SumX = 0;
				SumY = 0;
				x_2 = 0;
				x_y = 0;

				for (i = shizi_start_left; i >= shizi_start_left - 2; i--)
				{
					SumX = SumX + (i);
					SumY = SumY + leftline[i];
					x_2 = x_2 + (i) * (i);
					x_y = x_y + i * leftline[i];
				}

				AverageX = (float)SumX * 1.0 / AvaliableLines;
				AverageY = (float)SumY * 1.0 / AvaliableLines;
				AverageX_2 = (float)x_2 * 1.0 / AvaliableLines;
				AverageX_Y = (float)x_y * 1.0 / AvaliableLines;

				b_left = (AverageX_Y - AverageX * AverageY) * 100 / (AverageX_2 - AverageX * AverageX);
				slope_shizi_left = (float)b_left / 100;
				if (slope_shizi_left<-2.7 || slope_shizi_left>-0.5)
				{
					slope_shizi_left = -2.1;
				}
				for (i = shizi_start_left; i <= CAMERA_R_H - 1; i++)
				{
					leftline[i] = leftline[shizi_start_left] - slope_shizi_left * (shizi_start_left - i);
				}
			}
			if (shizi_start_right - 2 <= break_right_shizi)
			{
				for (i = 0; i <= CAMERA_R_H - 1; i++)
				{
					rightline[i] = leftline[i] + width[i];
				}
			}
			if (shizi_start_left - 2 <= break_left_shizi)
			{
				for (i = 0; i <= CAMERA_R_H - 1; i++)
				{
					leftline[i] = rightline[i] - width[i];
				}
			}
		}
	}

	if (shizi_count == 1)
	{
		send_num = 3;
	}
	else if (shizi_count == 3)
	{
		send_num = 14;
	}

	tiaobian_left = 0;
	tiaobian_right = 0;
	xierushizi_flag = 0;
	slope_xierushizi = 0;

	if ((rightline_flag == 1 || leftline_flag == 1) && (abs(break_left - break_right) > 5 || (break_left > 10 || break_right > 10)))     //环道和直入十字判断重叠       abs(break_left-break_right)>10！！！！！！！！！！！！！！！！
	{
		for (i = CAMERA_R_H - 3; i >= break_left + 2; i--)
		{
			if (leftline[i] >= leftline[i + 1] && leftline[i] >= leftline[i - 1] && leftline[i + 1] >= leftline[i + 2] && leftline[i - 1] >= leftline[i - 2])
			{
				tiaobian_left = i;
				break;
			}
		}
		for (i = CAMERA_R_H - 3; i >= break_right + 2; i--)                                  //寻找右边的跳变点
		{
			if (rightline[i] <= rightline[i + 1] && rightline[i] <= rightline[i - 1] && rightline[i + 1] <= rightline[i + 2] && rightline[i - 1] <= rightline[i - 2])
			{
				tiaobian_right = i;
				break;
			}
		}

		//************斜入十字判断*******************//
		if (shizi_count != 1 && shizi_count != 3)      //  !!!!!!     shizi_count!=1&&shizi_count!=3
		{
			if (rightline_flag == 1 && break_left - break_right > 5)          //5
			{
				if (tiaobian_right != 0)
				{
					slope1 = (float)(rightline[tiaobian_right] - rightline[CAMERA_R_H - 1]) / (CAMERA_R_H - 1 - tiaobian_right);
					if (tiaobian_right - break_right >= 5)
					{
						slope2 = (float)(rightline[tiaobian_right - 5] - rightline[tiaobian_right]) / 5;
					}
					else
					{
						slope2 = (float)(rightline[break_right] - rightline[tiaobian_right]) / (tiaobian_right - break_right);
					}

					if (abs(slope1 - slope2) >= 6 && slope1*slope2 < 0)////////&& (abs(slope1)<abs(slope2))
					{
						xierushizi_flag = 2;
						break_right = tiaobian_right;
					}
				}
				else
				{
					if (break_right <= 35)
					{
						slope_you_shizi = Least_Squares(&rightline[0], 5, CAMERA_R_H - 1);
					}
					else if (break_right <= 38 && break_right > 35)
					{
						slope_you_shizi = Least_Squares(&rightline[0], CAMERA_R_H - break_right, CAMERA_R_H - 1);
					}
					if (slope_you_shizi < 0)
					{
						xieshizi_left_tiaobian = 0;
						for (int i = rightline[39]; i >= 3; i--)
						{
							if (imgbuff[break_right + 2][i] - imgbuff[break_right + 2][i - 3] > Jump_threhold&&imgbuff[break_right + 2][i] > 150 && imgbuff[break_right + 2][i - 3] < 150)
							{
								xieshizi_left_tiaobian = i - 3;
								break;
							}
							else
							{
								xieshizi_left_tiaobian = 0;
							}
						}
						if (xieshizi_left_tiaobian == 0)
						{
							xierushizi_flag = 4;
						}
						else
						{
							xierushizi_flag = 0;
						}
					}
				}
			}

			if (break_right - break_left > 5 && leftline_flag == 1)                               //右转斜入十字
			{
				if (tiaobian_left != 0)
				{
					slope1 = (float)(leftline[tiaobian_left] - leftline[CAMERA_R_H - 1]) / (CAMERA_R_H - 1 - tiaobian_left);
					if (tiaobian_left - break_left >= 5)
					{
						slope2 = (float)(leftline[tiaobian_left - 5] - leftline[tiaobian_left]) / 5;
					}
					else
					{
						slope2 = (float)(leftline[break_left] - leftline[tiaobian_left]) / (tiaobian_left - break_left);
					}

					if (abs(slope1 - slope2) >= 6 && slope1*slope2 < 0)  ///////&& (abs(slope1)<abs(slope2))
					{
						xierushizi_flag = 1;
						break_left = tiaobian_left;           ////?????????????????????????????????????????????
					}
				}
				else
				{
					if (break_left <= 35)
					{
						slope_zuo_shizi = Least_Squares(&leftline[0], 5, CAMERA_R_H - 1);
					}
					else if (break_left <= 38 && break_left > 35)
					{
						slope_zuo_shizi = Least_Squares(&leftline[0], CAMERA_R_H - break_right, CAMERA_R_H - 1);
					}
					if (slope_zuo_shizi > 0)
					{
						xieshizi_right_tiaobian = 0;
						for (int i = leftline[39]; i <= 196; i++)
						{
							if (imgbuff[break_left + 2][i] - imgbuff[break_left + 2][i + 3] > Jump_threhold&&imgbuff[break_left + 2][i] > 150 && imgbuff[break_left + 2][i + 3] < 150)
							{
								xieshizi_right_tiaobian = i + 3;
								break;
							}
							else
							{
								xieshizi_right_tiaobian = 0;
							}
						}
						if (xieshizi_right_tiaobian == 0)
						{
							xierushizi_flag = 3;
						}
						else
						{
							xierushizi_flag = 0;
						}
					}
				}
			}
		}
	}

	//********斜入十字补线 *******// 
	if (xierushizi_flag == 1)                         //左斜入十字补线！
	{
		if (Last_line - tiaobian_left > 5)
		{
			slope_xieshizi_zuo = Least_Squares(&leftline[0], 5, tiaobian_left + 4);
		}
		else if (Last_line - tiaobian_left > 0 && Last_line - tiaobian_left <= 5)
		{
			slope_xieshizi_zuo = Least_Squares(&leftline[0], Last_line - tiaobian_left + 1, Last_line);
		}
		for (i = tiaobian_left + 2; i >= 0; i--)
		{
			leftline[i] = leftline[tiaobian_left + 2] - slope_xieshizi_zuo*(tiaobian_left - i + 2) / 10;      //tiaobian-i+2
		}
		if (break_right <= 35)
		{
			slope_xierushizi = Least_Squares(&rightline[0], CAMERA_R_H - break_right, CAMERA_R_H - 1);
			for (i = break_right + 2; i >= 0; i--)       //break_left+3
			{
				rightline[i] = rightline[break_right + 2] - slope_xierushizi / 10 * (break_right - i + 2);       ////转向不够加5
			}
		}
		else
		{
			for (i = break_right; i >= 0; i--)
			{
				rightline[i] = leftline[i] + width[i] + 12;
			}
		}
	}
	else if (xierushizi_flag == 2)                    //右斜入十字补线！
	{
		if (Last_line - tiaobian_right > 5)
		{
			slope_xieshizi_you = Least_Squares(&rightline[0], 5, tiaobian_right + 4);
		}
		else if (Last_line - tiaobian_right > 0 && Last_line - tiaobian_right <= 5)
		{
			slope_xieshizi_you = Least_Squares(&rightline[0], Last_line - tiaobian_right + 1, Last_line);
		}
		for (i = tiaobian_right + 2; i >= 0; i--)
		{
			rightline[i] = rightline[tiaobian_right + 2] - slope_xieshizi_you*(tiaobian_right - i + 2) / 10;   //tiaobian-i+2
		}
		if (break_left <= 35)
		{
			slope_xierushizi = Least_Squares(&leftline[0], CAMERA_R_H - break_left, CAMERA_R_H - 1);
			for (i = break_left + 2; i >= 0; i--)  //break_right+3
			{
				leftline[i] = leftline[break_left + 2] - slope_xierushizi / 10 * (break_left - i + 2);     //转向不够加5
			}
		}
		else
		{
			for (i = break_left; i >= 0; i--)
			{
				leftline[i] = rightline[i] - width[i] - 12;
			}
		}
	}
	else if (xierushizi_flag == 3)/////////车子以很倾斜的姿态左转进入十字
	{
		/*********************先找左右线的基准点************************/
		xieshizi_find_right = 0;
		xieshizi_youdiu_count = 0;
		xieshizi_break_right = 39;
		shizi_start_mid = leftline[CAMERA_R_H - 1] + 100;
		for (int i = Last_line; i >= 15; i--)    ////不能太往前，太靠前会扫描到环岛，左线就不对了
		{
			for (int j = 195; j >= shizi_start_mid; j--)
			{
				if (imgbuff[i][j] - imgbuff[i][j + 3] > Jump_threhold&&imgbuff[i][j] - imgbuff[i][j + 4] > Jump_threhold&&imgbuff[i][j] > 150 && imgbuff[i][j + 3] < 150)
				{
					if (xieshizi_youdiu_count == 0)
					{
						xieshizi_youdiu_count++;
						if (xieshizi_youdiu_count == 1)
						{
							xieshizi_find_right = i;
						}
					}
					rightline[i] = j + 3;
					xieshizi_break_right = 15;
					find_line_right = 1;
					break;
				}
				else
				{
					find_line_right = 0;
				}
			}
			if (find_line_right == 0 && xieshizi_youdiu_count == 1)
			{
				xieshizi_break_right = i + 1;
				break;
			}
		}
		if (xieshizi_youdiu_count == 1)
		{
			xieshizi_find_left = 0;
			xieshizi_zuodiu_count = 0;
			xieshizi_break_left = 39;
			for (int i = xieshizi_find_right; i >= 15; i--)
			{
				for (int j = rightline[xieshizi_find_right]; j >= 4; j--)
				{
					if (imgbuff[i][j] - imgbuff[i][j - 3] > Jump_threhold && imgbuff[i][j] - imgbuff[i][j - 4] > Jump_threhold&&imgbuff[i][j] > 150 && imgbuff[i][j - 3] < 150)
					{
						if (xieshizi_zuodiu_count == 0)
						{
							xieshizi_zuodiu_count++;
							if (xieshizi_zuodiu_count == 1)
							{
								xieshizi_find_left = i;
							}
						}
						leftline[i] = j - 3;
						xieshizi_break_left = 15;
						find_line_left = 1;
						break;
					}
					else
					{
						find_line_left = 0;
					}
				}
				if (find_line_left == 0 && xieshizi_zuodiu_count == 1)
				{
					xieshizi_break_left = i + 1;
					break;
				}
			}
		}
		else if (xieshizi_youdiu_count == 0)
		{
			if (break_left >= 5)
			{
				xieshizi_zuodiu_count = 0;
				xieshizi_find_left = 0;
				xieshizi_break_left = 39;
				for (int i = break_left - 2; i >= 0; i--)
				{
					for (int j = 199; j >= leftline[39]; j--)
					{
						if (imgbuff[i][j] - imgbuff[i][j - 3] > Jump_threhold&&imgbuff[i][j] > 150 && imgbuff[i][j - 3] < 150)
						{
							if (xieshizi_zuodiu_count == 0)
							{
								xieshizi_zuodiu_count++;
								if (xieshizi_zuodiu_count == 1)
								{
									xieshizi_find_left = i;
								}
							}
							leftline[i] = j - 3;
							xieshizi_break_left = 0;
							find_line_left = 1;
							break;
						}
						else
						{
							find_line_left = 0;
						}
					}
					if (xieshizi_zuodiu_count == 1 && find_line_left == 0)
					{
						xieshizi_break_left = i + 1;
						break;
					}
				}
			}
		}
		/*************斜入十字重新进行左右线往上跟踪****************/
		if (xieshizi_zuodiu_count == 1)
		{
			xieshizi_rebreak_left = 39;
			for (int i = xieshizi_break_left; i >= 0; i--) // 40
			{
				for (j = Tracking_NUM; j >= 0; j--) ///    18
				{
					if (leftline[i + 1] + j - Tracking_displacement >= 0 && leftline[i + 1] + j + Interval - Tracking_displacement < CAMERA_W) //   9
					{
						if (imgbuff[i][leftline[i + 1] + j + Interval - Tracking_displacement] - imgbuff[i][leftline[i + 1] + j - Tracking_displacement] > (Jump_threhold - 0.3 * (39 - i)) && imgbuff[i][leftline[i + 1] + j + Interval - Tracking_displacement] > 150 && imgbuff[i][leftline[i + 1] + j - Tracking_displacement] < 150) //  0.3
						{
							leftline[i] = leftline[i + 1] + j - Tracking_displacement;
							find_line_left = 1;
							xieshizi_rebreak_left = 0;
							break;
						}
						else
						{
							find_line_left = 0;
						}
					}
				}
				if (find_line_left == 0)
				{
					xieshizi_rebreak_left = i + 1;
					break;
				}
			}
		}
		if (xieshizi_youdiu_count == 1)
		{
			if (xieshizi_find_right - xieshizi_break_right >= 3)
			{
				xieshizi_you_restart = xieshizi_find_right - 3;
			}
			else if (xieshizi_find_right - xieshizi_break_right < 3 && xieshizi_find_right - xieshizi_break_right >= 0)
			{
				xieshizi_you_restart = xieshizi_break_right;
			}
			xieshizi_rebreak_right = 39;
			for (int i = xieshizi_you_restart; i >= 0; i--)
			{
				for (j = 0; j <= Tracking_NUM; j++) //  18
				{
					if (rightline[i + 1] + j - Interval - Tracking_displacement >= 0 && rightline[i + 1] + j - Tracking_displacement < CAMERA_W)
					{
						if (imgbuff[i][rightline[i + 1] + j - Interval - Tracking_displacement] - imgbuff[i][rightline[i + 1] + j - Tracking_displacement] > (Jump_threhold - 0.3 * (39 - i)) && imgbuff[i][rightline[i + 1] + j - Interval - Tracking_displacement] > 150 && imgbuff[i][rightline[i + 1] + j - Tracking_displacement] < 150)       // gai imgbuff[i][rightline[i + 1] + j - Tracking_displacement -Interval] -imgbuff[i][rightline[i + 1] + j - Tracking_displacement] >(Jump_threhold - 0.3 * (39 - i))
						{
							rightline[i] = rightline[i + 1] + j - Tracking_displacement;
							find_line_right = 1;
							xieshizi_rebreak_right = 0;
							break;
						}
						else
						{
							find_line_right = 0;
						}
					}
				}
				if (find_line_right == 0)
				{
					xieshizi_rebreak_right = i + 1;
					break;
				}
			}
		}

		//左线往后补线
		if (xieshizi_find_left - xieshizi_rebreak_left >= 5)
		{
			slpoe_xieshizi_leftback = Least_Squares(&leftline[0], 5, xieshizi_find_left);
			for (int i = xieshizi_find_left - 2; i <= 39; i++)
			{
				leftline[i] = leftline[xieshizi_find_left - 2] - slpoe_xieshizi_leftback * (xieshizi_find_left - 2 - i) / 10;
			}
		}
		else if (xieshizi_find_left - xieshizi_rebreak_left < 5 && xieshizi_find_left - xieshizi_rebreak_left>0)
		{
			slpoe_xieshizi_leftback = Least_Squares(&leftline[0], xieshizi_find_left - xieshizi_rebreak_left + 1, xieshizi_find_left);
			for (int i = xieshizi_find_left; i <= 39; i++)
			{
				leftline[i] = leftline[xieshizi_find_left] - slpoe_xieshizi_leftback * (xieshizi_find_left - i) / 10;
			}
		}
		//左线往前补线
		if (xieshizi_find_left - xieshizi_rebreak_left >= 5)
		{
			slpoe_xieshizi_leftfront = Least_Squares(&leftline[0], 5, xieshizi_rebreak_left + 4);
			for (int i = xieshizi_rebreak_left + 2; i >= 0; i--)
			{
				leftline[i] = leftline[xieshizi_rebreak_left + 2] - slpoe_xieshizi_leftfront * (xieshizi_rebreak_left + 2 - i) / 10;
			}
		}
		else   if (xieshizi_find_left - xieshizi_rebreak_left < 5 && xieshizi_find_left - xieshizi_rebreak_left>0)
		{
			slpoe_xieshizi_leftfront = Least_Squares(&leftline[0], xieshizi_find_left - xieshizi_rebreak_left + 1, xieshizi_find_left);
			for (int i = xieshizi_rebreak_left; i >= 0; i--)
			{
				leftline[i] = leftline[xieshizi_rebreak_left] - slpoe_xieshizi_leftfront * (xieshizi_rebreak_left - i) / 10;
			}
		}
		//右线补线
		if (xieshizi_youdiu_count == 1)
		{
			if (xieshizi_find_right - xieshizi_rebreak_right >= 5)
			{
				slpoe_xieshizi_rightback = Least_Squares(&rightline[0], 5, xieshizi_find_right);
				for (int i = xieshizi_find_right - 2; i <= 39; i++)
				{
					rightline[i] = rightline[xieshizi_find_right - 2] - slpoe_xieshizi_rightback * (xieshizi_find_right - 2 - i) / 10;
				}
				slpoe_xieshizi_rightfront = Least_Squares(&rightline[0], 5, xieshizi_rebreak_right + 4);
				for (int i = xieshizi_rebreak_right + 2; i >= 0; i--)
				{
					rightline[i] = rightline[xieshizi_rebreak_right + 2] - slpoe_xieshizi_rightfront * (xieshizi_rebreak_right + 2 - i) / 10;
				}
			}
			else
			{
				for (int i = 39; i >= 0; i--)
				{
					rightline[i] = leftline[i] + width[i];
				}
			}
		}
		else if (xieshizi_youdiu_count == 0)
		{
			for (int i = 39; i >= 0; i--)
			{
				rightline[i] = leftline[i] + width[i];
			}
		}
	}
	else if (xierushizi_flag == 4)     /////////车子以很倾斜的姿态右转进入十字
	{
		/*********************先找左右线的基准点************************/
		xieshizi_find_left = 0;
		xieshizi_zuodiu_count = 0;
		xieshizi_break_left = 39;
		shizi_start_mid = rightline[CAMERA_R_H - 1] - 100;
		for (int i = Last_line; i >= 15; i--)    ////不能太往前，太靠前会扫描到环岛，左线就不对了
		{
			for (int j = 1; j <= shizi_start_mid; j++)
			{
				if (imgbuff[i][j + 3] - imgbuff[i][j] > Jump_threhold&&imgbuff[i][j + 3] - imgbuff[i][j - 1] > Jump_threhold&&imgbuff[i][j + 3] > 150 && imgbuff[i][j] < 150)
				{
					if (xieshizi_zuodiu_count == 0)
					{
						xieshizi_zuodiu_count++;
						if (xieshizi_zuodiu_count == 1)
						{
							xieshizi_find_left = i;
						}
					}
					leftline[i] = j;
					xieshizi_break_left = 15;
					find_line_left = 1;
					break;
				}
				else
				{
					find_line_left = 0;
				}
			}
			if (find_line_left == 0 && xieshizi_zuodiu_count == 1)
			{
				xieshizi_break_left = i + 1;
				break;
			}
		}
		if (xieshizi_zuodiu_count == 1)
		{
			xieshizi_find_right = 0;
			xieshizi_youdiu_count = 0;
			xieshizi_break_right = 39;
			for (int i = xieshizi_find_left; i >= 15; i--)
			{
				for (int j = leftline[xieshizi_find_left]; j <= 195; j++)
				{
					if (imgbuff[i][j] - imgbuff[i][j + 3] > Jump_threhold && imgbuff[i][j] - imgbuff[i][j + 4] > Jump_threhold&&imgbuff[i][j] > 150 && imgbuff[i][j + 3] < 150)
					{
						if (xieshizi_youdiu_count == 0)
						{
							xieshizi_youdiu_count++;
							if (xieshizi_youdiu_count == 1)
							{
								xieshizi_find_right = i;
							}
						}
						rightline[i] = j + 3;
						xieshizi_break_right = 15;
						find_line_right = 1;
						break;
					}
					else
					{
						find_line_right = 0;
					}
				}
				if (find_line_right == 0 && xieshizi_youdiu_count == 1)
				{
					xieshizi_break_right = i + 1;
					break;
				}
			}
		}
		else if (xieshizi_zuodiu_count == 0)
		{
			if (break_right >= 5)
			{
				xieshizi_youdiu_count = 0;
				xieshizi_find_right = 0;
				xieshizi_break_right = 39;
				for (int i = break_right - 2; i >= 0; i--)
				{
					for (int j = 0; j <= rightline[39]; j++)
					{
						if (imgbuff[i][j] - imgbuff[i][j + 3] > Jump_threhold&&imgbuff[i][j] > 150 && imgbuff[i][j + 3] < 150)
						{
							if (xieshizi_youdiu_count == 0)
							{
								xieshizi_youdiu_count++;
								if (xieshizi_youdiu_count == 1)
								{
									xieshizi_find_right = i;
								}
							}
							rightline[i] = j + 3;
							xieshizi_break_right = 0;
							find_line_right = 1;
							break;
						}
						else
						{
							find_line_right = 0;
						}
					}
					if (xieshizi_youdiu_count == 1 && find_line_right == 0)
					{
						xieshizi_break_right = i + 1;
						break;
					}
				}
			}
		}
		/*************斜入十字重新进行左右线往上跟踪****************/
		if (xieshizi_zuodiu_count == 1)
		{
			xieshizi_rebreak_left = 39;
			for (int i = xieshizi_break_left; i >= 0; i--) // 40
			{
				for (j = Tracking_NUM; j >= 0; j--) ///    18
				{
					if (leftline[i + 1] + j - Tracking_displacement >= 0 && leftline[i + 1] + j + Interval - Tracking_displacement < CAMERA_W) //   9
					{
						if (imgbuff[i][leftline[i + 1] + j + Interval - Tracking_displacement] - imgbuff[i][leftline[i + 1] + j - Tracking_displacement] > (Jump_threhold - 0.3 * (39 - i)) && imgbuff[i][leftline[i + 1] + j + Interval - Tracking_displacement] > 150 && imgbuff[i][leftline[i + 1] + j - Tracking_displacement] < 150) //  0.3
						{
							leftline[i] = leftline[i + 1] + j - Tracking_displacement;
							find_line_left = 1;
							xieshizi_rebreak_left = 0;
							break;
						}
						else
						{
							find_line_left = 0;
						}
					}
				}
				if (find_line_left == 0)
				{
					xieshizi_rebreak_left = i + 1;
					break;
				}
			}
		}
		if (xieshizi_youdiu_count == 1)
		{
			if (xieshizi_find_right - xieshizi_break_right >= 3)
			{
				xieshizi_you_restart = xieshizi_find_right - 3;
			}
			else if (xieshizi_find_right - xieshizi_break_right < 3 && xieshizi_find_right - xieshizi_break_right >= 0)
			{
				xieshizi_you_restart = xieshizi_break_right;
			}
			xieshizi_rebreak_right = 39;
			for (int i = xieshizi_you_restart; i >= 0; i--)
			{
				for (j = 0; j <= Tracking_NUM; j++) //  18
				{
					if (rightline[i + 1] + j - Interval - Tracking_displacement >= 0 && rightline[i + 1] + j - Tracking_displacement < CAMERA_W)
					{
						if (imgbuff[i][rightline[i + 1] + j - Interval - Tracking_displacement] - imgbuff[i][rightline[i + 1] + j - Tracking_displacement] > (Jump_threhold - 0.3 * (39 - i)) && imgbuff[i][rightline[i + 1] + j - Interval - Tracking_displacement] > 150 && imgbuff[i][rightline[i + 1] + j - Tracking_displacement] < 150)       // gai imgbuff[i][rightline[i + 1] + j - Tracking_displacement -Interval] -imgbuff[i][rightline[i + 1] + j - Tracking_displacement] >(Jump_threhold - 0.3 * (39 - i))
						{
							rightline[i] = rightline[i + 1] + j - Tracking_displacement;
							find_line_right = 1;
							xieshizi_rebreak_right = 0;
							break;
						}
						else
						{
							find_line_right = 0;
						}
					}
				}
				if (find_line_right == 0)
				{
					xieshizi_rebreak_right = i + 1;
					break;
				}
			}
		}


		//右线往后补线
		if (xieshizi_find_right - xieshizi_rebreak_right >= 5)
		{
			slpoe_xieshizi_rightback = Least_Squares(&rightline[0], 5, xieshizi_find_right);
			for (int i = xieshizi_find_right - 2; i <= 39; i++)
			{
				rightline[i] = rightline[xieshizi_find_right - 2] - slpoe_xieshizi_rightback * (xieshizi_find_right - 2 - i) / 10;
			}
		}
		else  if (xieshizi_find_right - xieshizi_rebreak_right > 0 && xieshizi_find_right - xieshizi_rebreak_right < 5)
		{
			slpoe_xieshizi_rightback = Least_Squares(&rightline[0], xieshizi_find_right - xieshizi_rebreak_right + 1, xieshizi_find_right);
			for (int i = xieshizi_find_right; i <= 39; i++)
			{
				rightline[i] = rightline[xieshizi_find_right] - slpoe_xieshizi_rightback * (xieshizi_find_right - i) / 10;
			}
		}
		//右线往前补线

		if (xieshizi_find_right - xieshizi_rebreak_right >= 5)
		{
			slpoe_xieshizi_rightfront = Least_Squares(&rightline[0], 5, xieshizi_rebreak_right + 4);
			for (int i = xieshizi_rebreak_right + 2; i >= 0; i--)
			{
				rightline[i] = rightline[xieshizi_rebreak_right + 2] - slpoe_xieshizi_rightfront * (xieshizi_rebreak_right + 2 - i) / 10;
			}
		}
		else  if (xieshizi_find_right - xieshizi_rebreak_right > 0 && xieshizi_find_right - xieshizi_rebreak_right < 5)
		{
			slpoe_xieshizi_rightfront = Least_Squares(&rightline[0], xieshizi_find_right - xieshizi_rebreak_right + 1, xieshizi_find_right);
			for (int i = xieshizi_rebreak_right; i >= 0; i--)
			{
				rightline[i] = rightline[xieshizi_rebreak_right] - slpoe_xieshizi_rightfront * (xieshizi_rebreak_right - i) / 10;
			}
		}

		//左线补线
		if (xieshizi_zuodiu_count == 1)
		{
			if (xieshizi_find_left - xieshizi_rebreak_left >= 5)
			{
				slpoe_xieshizi_leftback = Least_Squares(&leftline[0], 5, xieshizi_find_left);
				for (int i = xieshizi_find_left - 2; i <= 39; i++)
				{
					leftline[i] = leftline[xieshizi_find_left - 2] - slpoe_xieshizi_leftback * (xieshizi_find_left - 2 - i) / 10;
				}
				slpoe_xieshizi_leftfront = Least_Squares(&leftline[0], 5, xieshizi_rebreak_left + 4);
				for (int i = xieshizi_rebreak_left + 2; i >= 0; i--)
				{
					leftline[i] = leftline[xieshizi_rebreak_left + 2] - slpoe_xieshizi_leftfront * (xieshizi_rebreak_left + 2 - i) / 10;
				}
			}
			else
			{
				for (int i = 39; i >= 0; i--)
				{
					leftline[i] = rightline[i] - width[i];
				}
			}
		}
		else if (xieshizi_zuodiu_count == 0)
		{
			for (int i = 39; i >= 0; i--)
			{
				leftline[i] = rightline[i] - width[i];
			}
		}
	}
	//      
	/*************************************************
	正常情况下的补线程序
	*************************************************/
	/*****************前行斜率补线*******************/
	if (xierushizi_flag == 0 && shizi_count != 1 && shizi_count != 3)    //shizi_count!=1&&shizi_count!=3
	{
		if (break_right <= 35 && break_right != 0)
		{
			slope = Least_Squares(&rightline[0], CAMERA_R_H - break_right, CAMERA_R_H - 1);
			for (i = break_right + 4; i >= 0; i--)
			{
				rightline[i] = -slope*(break_right - i + 4) / 10 + rightline[break_right + 4];
			}
		}

		if (break_left <= 35 && break_left != 0)
		{
			slope = Least_Squares(&leftline[0], CAMERA_R_H - break_left, CAMERA_R_H - 1);
			for (i = break_left + 4; i >= 0; i--)
			{
				leftline[i] = -slope*(break_left - i + 4) / 10 + leftline[break_left + 4];
			}
		}
	}

	/*****************对左线参考点多进行补线*******************/
	if (xierushizi_flag == 0 && shizi_count != 1 && shizi_count != 3)       //&&shizi_count!=1&&shizi_count!=3
	{
		if (leftline_flag == 1 && break_left <= 35 && break_right > 35)
		{
			slope = Least_Squares(&leftline[0], CAMERA_R_H - break_left, CAMERA_R_H - 1);
			for (i = break_left + 4; i >= 0; i--)
			{
				leftline[i] = -slope*(break_left - i + 4) / 10 + leftline[break_left + 4];
			}
			for (i = break_right; i >= 0; i--)
			{
				rightline[i] = leftline[i] + kuandu[i];
			}
		}
		/*****************对右线参考点多进行补线*******************/
		if (rightline_flag == 1 && break_right <= 35 && break_left > 35)
		{
			slope = Least_Squares(&rightline[0], CAMERA_R_H - break_right, CAMERA_R_H - 1);
			for (i = break_right + 4; i >= 0; i--)
			{
				rightline[i] = -slope*(break_right - i + 4) / 10 + rightline[break_right + 4];
			}
			for (i = break_left; i >= 0; i--)
			{
				leftline[i] = rightline[i] - kuandu[i];
			}
		}
	}

	/*************************************************
	开始判断赛道类型
	*************************************************/
	zhidao_flag = 0;
	long temp1 = 0;
	long temp2 = 0;
	for (i = CAMERA_R_H - 1; i >= 0; i--)
	{
		left_cha[i] = (midline[Last_line] - width[i] / 2) - leftline[i];
		right_cha[i] = (midline[Last_line] + width[i] / 2) - rightline[i];
		mid_cha[i] = midline[Last_line] - (leftline[i] + rightline[i]) / 2;         //100???? midline_last??
		temp1 = temp1 + mid_cha[i];
	}
	avg_front = temp1 / CAMERA_R_H;
	if (break_left > break_right)
	{
		valid_line = break_left;       //valid_line 有效行
	}
	else
	{
		valid_line = break_right;
	}
	for (int i = valid_line; i <= CAMERA_R_H - 1; i++)
	{
		temp2 = temp2 + abs(mid_cha[i] - avg_front);
	}
	fangcha = temp2 / (CAMERA_R_H - valid_line);
	fangcha_cha = fangcha - fangcha_old;
	fangcha_old = fangcha;

	//******直道判断******//
	if (fangcha <= 12 && xierushizi_flag == 0 && shizi_count != 1 && shizi_count != 3)//7  !!!!!!!!!!!!         shizi_flag_zhi==0 
	{
		//*****坡道直道****// 
		if (fangcha <= 8 && valid_line == 0)
		{
			podao_zhidao_flag = 1;
		}
		else
		{
			podao_zhidao_flag = 0;
		}
	}
	else
	{
		podao_zhidao_flag = 0;
	}


	/*************************************************
	开始计算前20行与后20行的偏差
	*************************************************/
	zhongxian_front[3] = zhongxian_front[2];
	zhongxian_front[2] = zhongxian_front[1];
	zhongxian_front[1] = zhongxian_front[0];

	long temp = 0;
	if (stop_flag == 0)         //正常行驶状态
	{
		temp = 0;
		for (i = CAMERA_R_H - 2; i >= 19; i--)
		{
			temp = temp + weight[i - 19] * (leftline[i] + rightline[i]) / 2;
		}
		zhongxian_front[0] = temp / 210;
	}

	error_steer[2] = error_steer[1];
	error_steer[1] = error_steer[0];
	error_steer[0] = 100 - zhongxian_front[0];
	//      /*****中线的平滑传递*****/
	//
	//              if(break_right==0 && break_left==0 && shizi_flag_zhi!=0 &&
	//              xierushizi_flag!=0)    //两边都未找到线，保持上一次的误差
	//              {
	//                error_steer[5]=error_steer[4];
	//                error_steer[4]=error_steer[3];
	//                error_steer[3]=error_steer[2];
	//                error_steer[2]=error_steer[1];
	//                error_steer[1]=error_steer[0];
	//              }
	//              else //正常情况下，进行参数的平滑传递
	//              {
	//                error_steer[5]=error_steer[4];
	//                error_steer[4]=error_steer[3];
	//                error_steer[3]=error_steer[2];
	//                error_steer[2]=error_steer[1];
	//                error_steer[1]=error_steer[0];
	//                error_steer[0]=100-zhongxian_front[0];
	//              }

  //    /**********    串口发送    **************/ ///
	uart_putchar(UART5, 0xff);
	for (int j = 0; j < CAMERA_R_H; j++)
	{
		for (int i = 0; i < CAMERA_W; i++)
		{
			if (imgbuff[j][i] == 0xff)
			{
				imgbuff[j][i] = 0xfe;
			}
			if (rightline[j] == i)
			{
				imgbuff[j][rightline[j]] = 0;
			}
			if (leftline[j] == i)
			{
				imgbuff[j][leftline[j]] = 0;
			}
			uart_putchar(UART5, imgbuff[j][i]);
		}
	}

	//LCD_Show_Number(80, 1, avg_distance);
	//LCD_Show_Number(50, 2, fache_distance);
	//LCD_Show_Number(50, 3, fache_finish_flag);
	//LCD_Show_Number(50, 4, speed_set);
	//LCD_Show_Number(50, 5, ruhuandao_flag);
	//LCD_Show_Number(50, 6, chuhuandao_flag);
	//LCD_Show_Number(80, 5, distance1);
	//LCD_Show_Number(80, 1, speed_set);
	//LCD_Show_Number(20, 2, shizi_ready_flag);
	//LCD_Show_Number(80, 3, shizi_chaoche_distance);
	//LCD_Show_Number(20, 3, shizi_chaoche_flag);
	//LCD_Show_Number(80, 4, break_right_shizi);
	//LCD_Show_Number(20, 4, break_left_shizi);
	//LCD_Show_Number(50, 5, fangcha);
	//LCD_Show_Number(20, 6, jieshouchaoche_flag);
	//LCD_Show_Number(80, 6, houche_ready_flag);
	//LCD_Show_Number(50, 3, huandao[huandao_count]);
	//LCD_Show_Number(50, 4, huandao_count);
	//LCD_Show_Number(50, 5, ruhuandao_flag);
	//LCD_Show_Number(50, 6, chuhuandao_flag);
	//LCD_Show_Number(50, 4, shizi_hui_flag);
}


void LP_image()
{
	uint8 *a_point, *b_point, *c_point;
	uint8 a, b, c, d, i, j;

	for (i = 0; i < CAMERA_R_H; i++)
	{
		for (j = 1; j < (CAMERA_W - 1); j++)
		{
			a_point = &imgbuff[i][j - 1];
			b_point = &imgbuff[i][j];
			c_point = &imgbuff[i][j + 1];

			a = *a_point;
			b = *b_point;
			c = *c_point;

			if (a >= b)
			{
				d = b;
				b = a;
				a = d;
			}
			if (a >= c)
			{
				d = c;
				c = a;
				a = d;
			}
			if (b >= c)
			{
				d = c;
				c = b;
				b = d;
			}
			*(b_point) = b;
		}
	}
}

//快速中值化滤波
void MedianFilter()
{
	int a, b, c, d;
	for (j = 0; j < Wide; j++)
	{
		Pic[0][j] = imgbuff[0][j];
		Pic[Height - 1][j] = imgbuff[Height - 1][j];
	}
	for (i = 1; i < Height - 1; i++)
	{
		Pic[i][0] = imgbuff[i][0];
		Pic[i][Wide - 1] = imgbuff[i][Wide - 1];
	}
	for (i = 1; i<Height - 1; i++)
	{
		j = 1;
		a = qic_Sort(imgbuff[i - 1][j - 1], imgbuff[i - 1][j], imgbuff[i - 1][j + 1]);
		b = qic_Sort(imgbuff[i][j - 1], imgbuff[i][j], imgbuff[i][j + 1]);
		c = qic_Sort(imgbuff[i + 1][j - 1], imgbuff[i + 1][j], imgbuff[i + 1][j + 1]);
		d = qic_Sort(a, b, c);
		Pic[i][1] = d;
		for (j = 2; j<Wide - 1; j++)
		{
			if (imgbuff[i - 1][j + 1] == imgbuff[i - 1][j - 2] && imgbuff[i][j + 1] == imgbuff[i][j - 2] && imgbuff[i + 1][j + 1] == imgbuff[i + 1][j - 2])
			{
				Pic[i][j] = d;
			}
			else
			{
				a = qic_Sort(imgbuff[i - 1][j - 1], imgbuff[i - 1][j], imgbuff[i - 1][j + 1]);
				b = qic_Sort(imgbuff[i][j - 1], imgbuff[i][j], imgbuff[i][j + 1]);
				c = qic_Sort(imgbuff[i + 1][j - 1], imgbuff[i + 1][j], imgbuff[i + 1][j + 1]);
				d = qic_Sort(a, b, c);
				Pic[i][j] = d;
			}
		}
	}
	for (int j = 0; j < CAMERA_R_H; j++)
	{
		for (int i = 0; i < CAMERA_W; i++)
		{
			imgbuff[j][i] = Pic[j][i];
		}
	}
}

//快速三元素以任意顺序排序
int qic_Sort(int a, int b, int c)
{
	if ((b >= a && b <= c) || (b <= a && b >= c))
	{
		return b;
	}
	else if ((a >= b &&  a >= c) || (a <= b && a <= c))
	{
		return c;
	}
	else if ((c >= a &&  c >= b) || (c <= a && c <= b))
	{
		return a;
	}
}

/*===================================================================
功能：跟踪
===================================================================*/
void xunxian(void)
{
	//<<<<<<<<<<<<<<<<<<<<标志清零>>>>>>>>>>>>>>>>>>>>//
	sao_zuo = 1;
	sao_you = 1;
	leftline_flag = 0;
	rightline_flag = 0;
	break_right = 40;
	break_left = 40;
	//<<<<<<<<<<<<<<<<<<<<确定基础寻线开始的点>>>>>>>>>>>>>>>>>>>>//
	if (mid_old <= 1 + Interval)                             //底行中线小于一定值时，不扫左线
	{
		sao_zuo = 0;
		mid_old = 1 + Interval;
	}
	else if (mid_old >= CAMERA_W - Interval - 2)
	{
		sao_you = 0;
		mid_old = CAMERA_W - Interval - 2;
	}
	//<<<<<<<<<<<<<<<<<<<<低三行左线>>>>>>>>>>>>>>>>>>>>//
	if (sao_zuo == 1)
	{
		for (i = mid_old; i >= Interval; i--)
		{
			if (imgbuff[Last_line][i] - imgbuff[Last_line][i - Interval] > yuzhi[Last_line])
			{
				leftline[Last_line] = i - Interval;
				leftline_flag = 1;
				break;
			}
			else
			{
				leftline_flag = 0;
			}
		}
	}
	//<<<<<<<<<<<<<<<<<<<<低三行右线>>>>>>>>>>>>>>>>>>>>//
	if (sao_you == 1)
	{
		for (i = mid_old; i <= CAMERA_W - Interval - 1; i++)
		{
			if (imgbuff[Last_line][i] - imgbuff[Last_line][i + Interval] > yuzhi[Last_line])
			{
				rightline[Last_line] = i + Interval;
				rightline_flag = 1;
				break;
			}
			else
			{
				rightline_flag = 0;
			}
		}
	}
	//<<<<<<<<<<<<<<<<<<<<最底行补线程序>>>>>>>>>>>>>>>>>>>>//
	if (leftline_flag == 1 && rightline_flag == 0)                        //底行补线根据宽度补线，除了十字外最低行不出现两边丢失
	{
		rightline[Last_line] = leftline[Last_line] + widths;
		if (rightline[Last_line] < CAMERA_W)
		{
			rightline[Last_line] = CAMERA_W;
		}
	}
	if (leftline_flag == 0 && rightline_flag == 1)
	{
		leftline[Last_line] = rightline[Last_line] - widths;
		if (leftline[Last_line] > 0)
		{
			leftline[Last_line] = 0;
		}
	}
	midline[Last_line] = (rightline[Last_line] + leftline[Last_line]) / 2;
	mid_old = midline[Last_line];                                      //记录上一次中线以便继续扫线
																	   //<<<<<<<<<<<<<<<<<<<<往前扫两行>>>>>>>>>>>>>>>>>>>>//
	find_line_left = 1;
	find_line_right = 1;
	//<<<<<<<<<<<<<<<<<<<<基本跟踪算法>>>>>>>>>>>>>>>>>>>>//
	//左线跟踪
	if (leftline_flag == 1)                                          //底行存在的情况下开始跟踪
	{
		for (i = Last_line - 1; i >= 0; i--)
		{
			for (j = Tracking_NUM; j >= 0; j--)
			{
				if (leftline[i + 1] + j - Tracking_displacement >= 0 && leftline[i + 1] + j + Interval - Tracking_displacement <= CAMERA_W - 1)      //防止数组溢出，跟踪从左线的右边九列到左边九列寻找下一行的左线
				{
					if (imgbuff[i][leftline[i + 1] + j + Interval - Tracking_displacement] - imgbuff[i][leftline[i + 1] + j - Tracking_displacement] > yuzhi[i])
					{
						leftline[i] = leftline[i + 1] + j - Tracking_displacement;
						find_line_left = 1;
						break_left = 0;
						break;
					}
					else
					{
						find_line_left = 0;
					}
				}
			}
			if (find_line_left == 0)
			{
				break_left = i + 1;
				break;              //跟踪不到后退出跟踪程序     
			}
		}
	}
	//右线跟踪
	if (rightline_flag == 1)
	{
		for (i = Last_line - 1; i >= 0; i--)
		{
			for (j = Tracking_NUM; j >= 0; j--)
			{
				if (rightline[i + 1] + j - Tracking_displacement - Interval >= 0 && rightline[i + 1] + j - Tracking_displacement <= CAMERA_W - 1)
				{
					if (imgbuff[i][rightline[i + 1] + j - Tracking_displacement - Interval] - imgbuff[i][rightline[i + 1] + j - Tracking_displacement] > yuzhi[i])
					{
						rightline[i] = rightline[i + 1] + j - Tracking_displacement;
						find_line_right = 1;
						break_right = 0;
						break;
					}
					else
					{
						find_line_right = 0;
					}
				}
			}
			if (find_line_right == 0)
			{
				break_right = i + 1;
				break;
			}
		}
	}
}

/*===================================================================
功能：补线
===================================================================*/
void buxian(void)
{
	//<<<<<<<<<<<<<<<<<<<<补线>>>>>>>>>>>>>>>>>>>>// 
	if (shizi_flag_zhi == 0 && qipao == 0 && podao_flag == 0 && xieru_shizi_flag == 0)
	{
		if (break_left <= 32)
		{
			if (break_left != 0)
			{
				b2[0] = Least_Squares(&leftline[0], 5, break_left + 7) / 10;
			}
			b2[4] = b2[3];
			b2[3] = b2[2];
			b2[2] = b2[1];
			b2[1] = b2[0];
			b = (b2[0] + b2[1] + b2[2] + b2[3] + b2[4]) / 5;
		}

		if (break_right <= 32)
		{
			if (break_right != 0)
			{
				b3[0] = Least_Squares(&rightline[0], 5, break_right + 7) / 10;
			}
			b3[4] = b3[3];
			b3[3] = b3[2];
			b3[2] = b3[1];
			b3[1] = b3[0];
			b1 = (b3[0] + b3[1] + b3[2] + b3[3] + b3[4]) / 5;
		}
		if (break_left <= 32 && break_right <= 32)
		{
			if (break_left != 0)
			{
				for (i = break_left + 1; i >= 0; i--)
				{
					leftline[i] = leftline[break_left + 2] - b*(break_left - i + 2);
				}
			}
			if (break_right != 0)
			{
				for (i = break_right + 1; i >= 0; i--)
				{
					rightline[i] = rightline[break_right + 2] - b1*(break_right - i + 2);
				}
			}
		}
		else if (break_left <= 32 && break_right > 32)
		{
			if (break_left != 0)
			{
				for (i = break_left + 1; i >= 0; i--)
				{
					leftline[i] = leftline[break_left + 2] - b*(break_left - i + 2);
				}
			}
			for (i = break_right; i >= 0; i--)
			{
				rightline[i] = leftline[i] + width[i];
			}
		}
		else if (break_right <= 32 && break_left > 32)
		{
			if (break_right != 0)
			{
				for (i = break_right + 1; i >= 0; i--)
				{
					rightline[i] = rightline[break_right + 2] - b1*(break_right - i + 2);
				}
			}
			for (i = break_left; i >= 0; i--)
			{
				leftline[i] = rightline[i] - width[i];
			}
		}
	}

	Midline = 0;
	piancha = 0;
	space = 0;
	zhuanxiang = 0;
	for (i = 39; i > 0; i--)
	{
		midline[i] = (leftline[i] + rightline[i]) / 2;
		with[i] = rightline[i] - leftline[i];
	}
	for (int j = CAMERA_R_H - 1; j >= CAMERA_R_H - qianzhan; j--)
	{
		Midline += midline[j] * (40 - j);
		space = space + 40 - j;
	}
	error_piancha[5] = error_piancha[4];
	error_piancha[4] = error_piancha[3];
	error_piancha[3] = error_piancha[2];
	error_piancha[2] = error_piancha[1];
	error_piancha[1] = error_piancha[0];
	error_piancha[0] = 100 - Midline / space;
	piancha = (error_piancha[5] + error_piancha[4] + error_piancha[3] + error_piancha[2] + error_piancha[1] + error_piancha[0]) / 6;
	//piancha=Kalman_Filter_mid(zhuanxiang);

	error_change[1] = error_change[0];
	error_change[0] = piancha;
	error_change[2] = error_change[0] - error_change[1];

	if (shizi_flag_zhi == 1)
	{
		if (piancha > 5)
		{
			piancha = 5;
		}
		if (piancha < -5)
		{
			piancha = -5;
		}
	}
	if (piancha > 80)
	{
		piancha = 80;
	}
	if (piancha < -80)
	{
		piancha = -80;
	}
	piancha_result = piancha;
	//<<<<<<<<<<<<<<<<<<<<串口发送程序>>>>>>>>>>>>>>>>>>>>//
	uart_putchar(UART5, 0xff);
	for (int j = 0; j < CAMERA_R_H; j++)
	{
		for (int i = 0; i < CAMERA_W; i++)
		{
			if (imgbuff[j][i] == 0xff)
			{
				imgbuff[j][i] = 0xfe;
			}
			if (rightline[j] == i)
			{
				imgbuff[j][rightline[j]] = 0;
			}
			if (leftline[j] == i)
			{
				imgbuff[j][leftline[j]] = 0;
			}
			uart_putchar(UART5, imgbuff[j][i]);
		}
	}

}