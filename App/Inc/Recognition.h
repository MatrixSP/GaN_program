#ifndef _RECOGNITION_H
#define _RECOGNITION_H
/// <summary>
/// 采集初始化
/// </summary>
void Main_ADC_INIT();
/// <summary>
/// 电感采集
/// </summary>
void AD_Collect();
/// <summary>
/// 模式识别，1->直道 3->前面有圆环 4->入圆环 5->圆环内 6->出圆环状态 7->已出圆环
/// </summary>
void ModeRecognition(void);
/// <summary>
/// 偏差计算函数
/// </summary>
void Error_Calculation(void);
/// <summary>
/// 模式控制，1->直道 3->前面有圆环 4->入圆环 5->圆环内 6->出圆环状态 7->已出圆环
/// </summary>
void ModeControl();
void Lose_Pro();

#endif

