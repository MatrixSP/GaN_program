#ifndef _RECOGNITION_H
#define _RECOGNITION_H
/// <summary>
/// �ɼ���ʼ��
/// </summary>
void Main_ADC_INIT();
/// <summary>
/// ��вɼ�
/// </summary>
void AD_Collect();
/// <summary>
/// ģʽʶ��1->ֱ�� 3->ǰ����Բ�� 4->��Բ�� 5->Բ���� 6->��Բ��״̬ 7->�ѳ�Բ��
/// </summary>
void ModeRecognition(void);
/// <summary>
/// ƫ����㺯��
/// </summary>
void Error_Calculation(void);
/// <summary>
/// ģʽ���ƣ�1->ֱ�� 3->ǰ����Բ�� 4->��Բ�� 5->Բ���� 6->��Բ��״̬ 7->�ѳ�Բ��
/// </summary>
void ModeControl();
void Lose_Pro();

#endif

