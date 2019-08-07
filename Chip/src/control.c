#include "include.h"
/*===================================================================
���ܣ���������
===================================================================*/
extern float AG;       //ֱ���Ƕ�
extern float AP;       //ֱ���� P
extern float AD;       //ֱ���� D
extern float SS;       //�ٶ�
extern float SP;       //�ٶȻ� P
extern float SI;       //�ٶȻ� I
extern float SD;       //�ٶȻ� I
extern uint32 Distance;
//ֱ����
float ANGLE_OFFSET = 0;
float P_ANGLE = 0;
float D_ANGLE = 0;
//�ٶȻ�         
int16 Speed_Set = 0;
float P_SPEED = 0;
float D_SPEED = 0;
float I_SPEED = 0;
extern float DIRC_P, DIRC_D, DIRC_I;
//float DIRC_P = 0;// 180;
//float DIRC_D = 0;
/*===================================================================
���ܣ��������˲�
Kalman�˲�0.001   0.003    0.5   0.005
===================================================================*/
float angle, angle_dot;         //�ⲿ��Ҫ���õı���
const float Q_angle = 0.5, Q_gyro = 2.5, R_angle = 7.5, dt = 0.005;
static float P[2][2] =
{
  { 1, 0 },
  { 0, 1 }
};
static float Pdot[4] = { 0,0,0,0 };
static const char C_0 = 1;
static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
void Kalman_Filter(float angle_m, float gyro_m)          //gyro_m:gyro_measure
{
	angle += (gyro_m - q_bias) * dt;//״̬����
	// �������
	Pdot[0] = Q_angle - P[0][1] - P[1][0];
	Pdot[1] = -P[1][1];
	Pdot[2] = -P[1][1];
	Pdot[3] = Q_gyro;
	P[0][0] += Pdot[0] * dt;
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;
	angle_err = angle_m - angle;//�в�
	//�������� 
	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];
	E = R_angle + C_0 * PCt_0;
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];
	P[0][0] -= K_0 * t_0;
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;
	//���ݸ���  
	angle += K_0 * angle_err;
	q_bias += K_1 * angle_err;
	angle_dot = gyro_m - q_bias;
}
/*===================================================================
���ܣ�ֱ���Ƕȼ���
===================================================================*/
float Pitch = 0.0;                              //���ٶȸ�����
void getangle(float Ax, float Ay, float Az)
{
	if ((Ay >= 0.0) && (Az >= 0.0))
	{
		Pitch = atan(Ay / Az)*180.0 / 3.14;//����Ҫת��Ϊ�Ƕ�
	}
	else if ((Ay >= 0.0) && (Az < 0.0))
	{
		Pitch = atan(Ay / Az)*180.0 / 3.14 + 180.0;
	}
	else if ((Ay < 0.0) && (Az > 0.0))
	{
		Pitch = atan(Ay / Az)*180.0 / 3.14 + 360.0;
	}
	else if ((Ay < 0.0) && (Az < 0.0))
	{
		Pitch = atan(Ay / Az)*180.0 / 3.14 + 180.0;
	}
	Pitch = Pitch - PITCH_OFFSET;
	Pitch = -Pitch;
	if (Pitch <= 0)
		Pitch = 0;
	if (Pitch >= 90)
		Pitch = 90;
}
double MMA_x = 0.0, MMA_y = 0.0, MMA_z = 0.0;
int16 GYRO_x = 0, GYRO_y = 0, GYRO_z = 0, GYRO_z_pre = 0;
void AD_Calculate(void)
{
	GYRO_z_pre = GYRO_z;
	MMA_x = Get_mma8451_once('X');
	MMA_y = Get_mma8451_once('Y');
	MMA_z = Get_mma8451_once('Z');
	getangle((float)MMA_x, (float)MMA_y, (float)MMA_z);
	GYRO_x = 0.068 *Get_Gyro(5, 'X');
	GYRO_y = 0.07*Get_Gyro(5, 'Y');
	GYRO_z = 0.07*Get_Gyro(5, 'Z');
	Kalman_Filter((float)Pitch, (int16)-GYRO_x);
}
/*===================================================================
���ܣ�ֱ������
��������λ��ʽPD������
===================================================================*/
float nAngleControlOut = 0;
float speed_Start;
void Speed_Calculate(float angle, float angle_dot)
{
	speed_Start = (angle - ANGLE_OFFSET)* P_ANGLE + angle_dot * D_ANGLE;  //ֱ��ʱ��Ҫ���ٶ�
	if (speed_Start > MOTOR_MAX_Z) speed_Start = MOTOR_MAX_Z;
	if (speed_Start < MOTOR_MIN_F) speed_Start = MOTOR_MIN_F;
	nAngleControlOut = speed_Start;
}
/*===================================================================
���ܣ�����������
===================================================================*/
uint8 DFLAG = 0, DFLAG1 = 0;
int16 temp_Left = 0, temp_Right = 0;
int distance_podao = 0;
int distance = 0;
int distances = 0;
int distance_chuhuan = 0;
int distance_zhangai = 0;
float nSpeed;
uint8 yuanhuan_flag = 0;
float yuanhuan_distance = 0;
#define count_MAX 1000
extern int flag_count_S;//����־λ
extern int distance_test;//����
extern int Trace;
char stoop = 0;
int distance_stoop = 0;
extern int flag_stop;
extern int Speed_Scan;
float SpeedGet(void)
{
	nSpeed = DMA_count_get(DMA_CH1);
	DMA_count_reset(DMA_CH1);
	if (flag_count_S == 1)
	{
		distance_test += nSpeed;
	}
	else
	{
		distance_test = 0;
	}
	if (flag_stop > 0)
	{
		distance_stoop += nSpeed;
	}
	if(distance_stoop >= 50)
	{
		stoop++;
		flag_stop = 0;
		distance_stoop = 0;
	}
	return nSpeed;
}
/*===================================================================
���ܣ��ٶȿ���
������������ʽPI������
===================================================================*/
int8 nSpeedControlPeriod = 0;
uint8 nSpeedControlCount = 0;
float nSpeedError_now = 0;
float nSpeedError_old = 0;
float nP = 0, nI = 0, nD = 0, g_fSpeedControlIntegral = 0;
float Speed_Old = 0, Speed_New = 0;
float nSpeedControlOut = 0, nSpeedControlOut1 = 0;
int I_limite = 20000;
void SpeedControl(void)
{
	nSpeedError_now = Speed_Set - nSpeed;
	nD = nSpeedError_now - nSpeedError_old;
	nSpeedError_old = nSpeedError_now;
	nP = nSpeedError_now * P_SPEED;
	nI = nSpeedError_now * I_SPEED;
	g_fSpeedControlIntegral += nI;
	if (g_fSpeedControlIntegral >= I_limite)
	{
		g_fSpeedControlIntegral = I_limite;
	}
	if (g_fSpeedControlIntegral <= -I_limite)
	{
		g_fSpeedControlIntegral = -I_limite;
	}
	Speed_Old = Speed_New;
	Speed_New = nP + g_fSpeedControlIntegral + nD * D_SPEED;
}
void SpeedControlOutput(void)       //���ٶȾ���20��
{

	nSpeedControlOut1 = Speed_New - Speed_Old;
	nSpeedControlOut = nSpeedControlOut1*(nSpeedControlPeriod + 1) / SPEED_CONTROL_COUNT + Speed_Old;
	/*
	if (nSpeedControlOut >= 0)
	{
		nSpeedControlOut = 10000 - nSpeedControlOut;
		gpio_set(PTC12, 1);
		FTM_PWM_Duty(FTM0, FTM_CH1, nSpeedControlOut);
	}
	else if (nSpeedControlOut < 0)
	{
		nSpeedControlOut = 10000 + nSpeedControlOut;
		gpio_set(PTC12, 0);
		FTM_PWM_Duty(FTM0, FTM_CH1, nSpeedControlOut);
	}
	*/
	nSpeedControlOut = 10000 - nSpeedControlOut;
	if (nSpeedControlOut >= MOTOR_DEAD_VAL)
	{
		nSpeedControlOut = MOTOR_DEAD_VAL;
	}
	else if (nSpeedControlOut <= MOTOR_MAX_VAL)
	{
		nSpeedControlOut = MOTOR_MAX_VAL;
	}
	FTM_PWM_Duty(FTM0, FTM_CH1, nSpeedControlOut);
}
/*===================================================================
���ܣ��������
����������ɢ��λ��ʽPD������
===================================================================*/
int8 nDirectionControlPeriod = 0;
uint8 nDirectionControlCount = 0;
float nDirControlOut = 0;
float nDirControlOutNew = 0, nDirControlOutOld = 0;
float piancha;
float Ipiancha = 0;
extern  int Trace;//����Ԫ�ر�־
void DirectionControl(void)
{
	float P, I, D;
	GYRO_y = 0.07*Get_Gyro(5, 'Y') - 10;
	Ipiancha += piancha;
	if (Ipiancha >= 10000)
	{
		Ipiancha = 10000;
	}
	if (Ipiancha <= -10000)
	{
		Ipiancha = -10000;
	}
	P = DIRC_P;
	D = DIRC_D;
	I = DIRC_I;
	nDirControlOutOld = nDirControlOutNew;
	nDirControlOutNew = piancha*P - GYRO_y*D + Ipiancha * I;
}
void DirectionControlOutput(void)                                                       //ת�򻺳崦��
{
	int16 Value = 0;
	Value = nDirControlOutNew - nDirControlOutOld;
	nDirControlOut = Value * (nDirectionControlPeriod + 1) / 2 + nDirControlOutOld;
	nDirControlOut = 4500 + nDirControlOut;
	if (nDirControlOut >= STEER_MAX)nDirControlOut = STEER_MAX;
	if (nDirControlOut <= STEER_MIN)nDirControlOut = STEER_MIN;
	FTM_PWM_Duty(FTM2, FTM_CH0, nDirControlOut);
}
/*===================================================================
���ܣ�������
��������λ��ʽPI������
===================================================================*/
float Power_Set = 28;
float Power_P = 8;
float Power_I = 1.5;
#define Power_ILimition 6666
//float Power_Set = 21;
//float Power_P = 8;
//float Power_I = 1;
//#define Power_ILimition 10000
float BatteryVot = 10;//5.87;  //12; 430J
#define POWER_MAX 9000
#define POWER_MIN 0
float Amp = 0;
float Vol = 0;
float Vcc = 0;
float Power = 0;
float AmpL = 0;
float VolL = 0;
float VccL = 0;
float E_Power = 0;
float I_Power = 0;
int Power_PWM = 0;
int Charge_Finish = 0;
extern uint8 count_1ms;
void SetPower(void)
{
	if (VccL < BatteryVot)
	{
		I_Power += E_Power;
		if (I_Power >= Power_ILimition)
		{
			I_Power = Power_ILimition;
		}
		if (I_Power <= -Power_ILimition)
		{
			I_Power = -Power_ILimition;
		}
		Power_PWM = E_Power * Power_P + I_Power * Power_I;

		if (Power_PWM > POWER_MAX) Power_PWM = POWER_MAX;
		if (Power_PWM < POWER_MIN) Power_PWM = POWER_MIN;

		FTM_PWM_Duty(FTM1, FTM_CH0, Power_PWM);
	}
	else
	{
		DisableInterrupts;
		Charge_Finish = 1;
		count_1ms = 0;
		FTM_PWM_Duty(FTM1, FTM_CH0, 0);
	}
}