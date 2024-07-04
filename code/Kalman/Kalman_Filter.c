/*
 * Kalman_Filter.c
 *
 *  Created on: 2023��5��15��
 *      Author: wzl
 */

#include "Kalman_Filter.h"

//���忨�����ṹ�����
KalmanInfo kalman_motor_speedback;       // ��������ٶ��˲�
KalmanInfo kalman_imu_yaw;               // ƫ���Ǽ��ٶ��˲�


//��ʼ���������˲��ṹ��
void Kalman_Filter_Init(KalmanInfo *KalmanInfo_Structure)
{
    KalmanInfo_Structure->LastP = 0.02f;
    KalmanInfo_Structure->Now_P = 0.0f;
    KalmanInfo_Structure->out = 0.0f;
    KalmanInfo_Structure->Kg = 0.0f;
    KalmanInfo_Structure->Q = 0.001f;
    KalmanInfo_Structure->R = 0.58f;
    return;
}


//-------------------------------------------------------------------------------------------------------------------
// �������     �������˲�����
// ����˵��     info            �������˲��ṹ��ָ��
// ����˵��     new_value       �µĲ���ֵ
// ���ز���     �˲���Ĺ���ֵ
//-------------------------------------------------------------------------------------------------------------------
float Kalman_Filter_Fun(KalmanInfo *info, float new_value)
{
    //����Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
    info->Now_P = info->LastP + info->Q;
    //���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
    info->Kg = info->Now_P / (info->Now_P + info->R);
    //��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
    info->out = info->out + info->Kg * (new_value -info->out);//��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
    //����Э�����: ���ε�ϵͳЭ����� info->LastP Ϊ��һ������׼����
    info->LastP = (1-info->Kg) * info->Now_P;
    return info->out;
}


