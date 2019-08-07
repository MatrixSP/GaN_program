#ifndef _CONTROL_H
#define _CONTROL_H

#define SPEED_CONTROL_COUNT  25
#define DIRECTION_CONTROL_COUNT   2

#define PITCH_OFFSET            182             //加速度计角度整零

#define MOTOR_DEAD_VAL 10000
#define MOTOR_MAX_VAL 4000

#define MOTOR_MAX_Z  9600
#define MOTOR_MIN_Z  0

#define MOTOR_MAX_F  0
#define MOTOR_MIN_F  -9600

#define MOTOR_MAX_SPEED 7000
#define MOTOR_MIN_SPEED -7000

#define STEER_MAX  5300
#define STEER_MIN  3700

void Kalman_Filter(float angle_m, float gyro_m);
void AD_Calculate(void);
void getangle(float Ax, float Ay, float Az);
void Speed_Calculate(float angle, float angle_dot);
float SpeedGet(void);
void SpeedControl(void);
void SpeedControlOutput(void);
void DirectionControl(void);
void DirectionControlOutput(void);
void MotorOutput(void);
void SetMotorVoltage(void);
#endif