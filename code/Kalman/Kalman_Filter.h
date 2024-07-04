/*
 * Kalman_Filter.h
 *
 *  Created on: 2023��5��15��
 *      Author: wzl
 */

#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H


//Kalman Filter parameter
typedef struct 
{
    float LastP;    //�ϴι���Э���� ��ʼ��ֵΪ0.02
    float Now_P;    //��ǰ����Э���� ��ʼ��ֵΪ0
    float out;      //�������˲������ ��ʼ��ֵΪ0
    float Kg;       //���������� ��ʼ��ֵΪ0
    float Q;        //��������Э���� ��ʼ��ֵΪ0.001
    float R;        //�۲�����Э���� ��ʼ��ֵΪ0.543
}KalmanInfo;


// �����������˲����ṹ�����
extern KalmanInfo kalman_motor_speedback;
extern KalmanInfo kalman_imu_yaw;


//==================================================Kalman ��������==================================================
void   Kalman_Filter_Init       (KalmanInfo *KalmanInfo_Structure);
float  Kalman_Filter_Fun        (KalmanInfo *info, float new_value);
//==================================================Kalman ��������==================================================

#endif
