/*
 * icm20602_data_handle.h
 *
 *  Created on: 2023��3��16��
 *      Author: wzl
 */

#ifndef CODE_ICM20602_DATA_HANDLE_H_
#define CODE_ICM20602_DATA_HANDLE_H_

//����ͷ�ļ�
#include "zf_device_icm20602.h"
#include "cpu0_main.h"


// ����imuԭʼ���ݽṹ��
typedef struct
{
    float a_x;
    float a_y;
    float a_z;
    float g_x;
    float g_y;
    float g_z;
}Gyroscope_g_and_a_data;


// ������̬�ǲ����ṹ��
typedef struct
{
    float yaw;
    float roll;
    float pitch;
    bool  Flag_handle;
}Gyroscope_attitude_Angle;


typedef struct
{
    int16_t Xdata;
    int16_t Ydata;
    int16_t Zdata;
}gyro_param_t;


//�ⲿ������������̬�ǽṹ��
extern Gyroscope_attitude_Angle Gyroscope_attitude_Angle_data_get;
//�ⲿ�������������ݽṹ��
extern Gyroscope_g_and_a_data Gyroscope_g_and_a_data_get;
//�ⲿ������������Ʈ�������ݽṹ��
extern gyro_param_t GyroOffset;


//==================================================ICM20602_DATA_HANDLE ��������==================================================
void icm20602_pose_init                 (void);                                //��̬�ǽ����ʼ��
void gyroOffsetInit                     (void);                                //��������Ư��ʼ��
void icm20602_attitude_Angle_handle     (void);                                //��̬�ǽ���
void icm20602_attitude_Angle_Timer      (void);                                //�̼߳�ʱ
void IMUupdate (Gyroscope_g_and_a_data Gyroscope_g_and_a_data_get_t, Gyroscope_attitude_Angle *Gyroscope_attitude_Angle_data_get_t);
//==================================================ICM20602_DATA_HANDLE ��������==================================================

#endif
