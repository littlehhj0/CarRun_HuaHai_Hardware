/*
 * icm20602_data_handle.c
 *
 *  Created on: 2023��3��15��
 *      Author: wzl
 */


//����ͷ�ļ�
#include "icm20602_data_handle.h"
#include "Kalman/Kalman_Filter.h"
#include "Ifx_LutAtan2F32.h"
#include "zf_driver_gpio.h"


//������������̬�ǽṹ��
Gyroscope_attitude_Angle Gyroscope_attitude_Angle_data_get;
//�ⲿ�������������ݽṹ��
Gyroscope_g_and_a_data Gyroscope_g_and_a_data_get;
//������У׼ֵ�ṹ��
gyro_param_t GyroOffset;



//-------------------------------------------------------------------------------------------------------------------
// �������     �����������ʼ��
// ����˵��     void
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void icm20602_pose_init(void)
{
    //��ʼ��������icm20602
    while(icm20602_init());
    //��ƣ�˵����ʼ���ɹ�
    gpio_init(P20_8, GPO, 0, GPO_PUSH_PULL);

    //��ʼ����ȡ���ݽṹ��
    Gyroscope_g_and_a_data_get.a_x = 0;
    Gyroscope_g_and_a_data_get.a_y = 0;
    Gyroscope_g_and_a_data_get.a_z = 0;
    Gyroscope_g_and_a_data_get.g_x = 0;
    Gyroscope_g_and_a_data_get.g_y = 0;
    Gyroscope_g_and_a_data_get.g_z = 0;

    //��������Ʈ��ʼ��
    gyroOffsetInit();
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ��������̬�ǽ������
// ����˵��     void
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void icm20602_attitude_Angle_Timer(void)
{
    static uint16_t num = 0;
    num++;

    if(num >= 5)
    {
        Gyroscope_attitude_Angle_data_get.Flag_handle = true;
        num = 0;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ��������̬�ǽ������
// ����˵��     void
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void icm20602_attitude_Angle_handle(void)
{
    if(Gyroscope_attitude_Angle_data_get.Flag_handle)
    {
        //��������
        icm20602_get_acc();
        Gyroscope_g_and_a_data_get.a_x = icm20602_acc_transition(icm20602_acc_x);
        Gyroscope_g_and_a_data_get.a_y = icm20602_acc_transition(icm20602_acc_y);
        Gyroscope_g_and_a_data_get.a_z = icm20602_acc_transition(icm20602_acc_z);
        icm20602_get_gyro();
        Gyroscope_g_and_a_data_get.g_x = icm20602_gyro_transition(icm20602_gyro_x - GyroOffset.Xdata) * PI / 180;
        Gyroscope_g_and_a_data_get.g_y = icm20602_gyro_transition(icm20602_gyro_y - GyroOffset.Ydata) * PI / 180;
        Gyroscope_g_and_a_data_get.g_z = icm20602_gyro_transition(icm20602_gyro_z - GyroOffset.Zdata) * PI / 180;

        // ��̬�ǽ���
        IMUupdate(Gyroscope_g_and_a_data_get, &Gyroscope_attitude_Angle_data_get);
        // �����������ݽ����˲�
        Gyroscope_attitude_Angle_data_get.yaw = Kalman_Filter_Fun(&kalman_imu_yaw, Gyroscope_attitude_Angle_data_get.yaw);

        //��־λ����
        Gyroscope_attitude_Angle_data_get.Flag_handle = false;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ��������Ư��ʼ��,ͨ���ɼ�һ���������ֵ�������������ƫ��ֵ;���� �����Ƕ�ȡ������ - ��Ʈֵ������ȥ�����ƫ������
// ����˵��     void
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void gyroOffsetInit(void)
{
    GyroOffset.Xdata = 0;
    GyroOffset.Ydata = 0;
    GyroOffset.Zdata = 0;
    for (uint16_t i = 0; i < 100; ++i)
    {
        icm20602_get_gyro();
        GyroOffset.Xdata += icm20602_gyro_x;
        GyroOffset.Ydata += icm20602_gyro_y;
        GyroOffset.Zdata += icm20602_gyro_z;
        system_delay_ms(5);
    }

    GyroOffset.Xdata /= 100;
    GyroOffset.Ydata /= 100;
    GyroOffset.Zdata /= 100;
}


//==================================================��Ԫ������==================================================
#define Kp 10.0f                            // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.004f                           // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.005f                        // һ���������

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;       // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;      // scaled integral error

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������Ԫ�����㺯��
// ����˵��     �õ��Ľ��ٶ���Ϣ�ͼ��ٶ���Ϣ
// ���ز���     ��Ԫ����̬λ��
//-------------------------------------------------------------------------------------------------------------------
void IMUupdate(Gyroscope_g_and_a_data Gyroscope_g_and_a_data_get_t, Gyroscope_attitude_Angle *Gyroscope_attitude_Angle_data_get_t)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    // �Ȱ���Щ�õõ���ֵ���
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    // float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    // float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    if(Gyroscope_g_and_a_data_get_t.a_x*Gyroscope_g_and_a_data_get_t.a_y*Gyroscope_g_and_a_data_get_t.a_z == 0)
        return;

    // ���ٶȼƲ�������������(��������ϵ)
    norm = sqrtf(Gyroscope_g_and_a_data_get_t.a_x*Gyroscope_g_and_a_data_get_t.a_x +
                 Gyroscope_g_and_a_data_get_t.a_y*Gyroscope_g_and_a_data_get_t.a_y +
                 Gyroscope_g_and_a_data_get_t.a_z*Gyroscope_g_and_a_data_get_t.a_z);
    Gyroscope_g_and_a_data_get_t.a_x = Gyroscope_g_and_a_data_get_t.a_x / norm;
    Gyroscope_g_and_a_data_get_t.a_y = Gyroscope_g_and_a_data_get_t.a_y / norm;
    Gyroscope_g_and_a_data_get_t.a_z = Gyroscope_g_and_a_data_get_t.a_z / norm;

    // ��Ԫ���Ƴ���ʵ����������(��������ϵ)
    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // ������
    ex = (Gyroscope_g_and_a_data_get_t.a_y*vz - Gyroscope_g_and_a_data_get_t.a_z*vy);                                             //�������������õ���־������
    ey = (Gyroscope_g_and_a_data_get_t.a_z*vx - Gyroscope_g_and_a_data_get_t.a_x*vz);
    ez = (Gyroscope_g_and_a_data_get_t.a_x*vy - Gyroscope_g_and_a_data_get_t.a_y*vx);

    // ���������Ϊ���ٶ�
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    // ���ٶȲ���
    Gyroscope_g_and_a_data_get_t.g_x = Gyroscope_g_and_a_data_get_t.g_x + Kp*ex + exInt;        //�����PI�󲹳��������ǣ����������Ư��
    Gyroscope_g_and_a_data_get_t.g_y = Gyroscope_g_and_a_data_get_t.g_y + Kp*ey + eyInt;
    Gyroscope_g_and_a_data_get_t.g_z = Gyroscope_g_and_a_data_get_t.g_z + Kp*ez + ezInt;        //�����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�

    // ������Ԫ��
    q0 = q0 + (-q1*Gyroscope_g_and_a_data_get_t.g_x - q2*Gyroscope_g_and_a_data_get_t.g_y - q3*Gyroscope_g_and_a_data_get_t.g_z)*halfT;
    q1 = q1 + (q0*Gyroscope_g_and_a_data_get_t.g_x + q2*Gyroscope_g_and_a_data_get_t.g_z - q3*Gyroscope_g_and_a_data_get_t.g_y)*halfT;
    q2 = q2 + (q0*Gyroscope_g_and_a_data_get_t.g_y - q1*Gyroscope_g_and_a_data_get_t.g_z + q3*Gyroscope_g_and_a_data_get_t.g_x)*halfT;
    q3 = q3 + (q0*Gyroscope_g_and_a_data_get_t.g_z + q1*Gyroscope_g_and_a_data_get_t.g_y - q2*Gyroscope_g_and_a_data_get_t.g_x)*halfT;

    // ��λ����Ԫ��
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

    // ��Ԫ������ŷ����
    Gyroscope_attitude_Angle_data_get_t->yaw = atan2f(2 * (q1*q2 + q0*q3), -2*q2*q2 - 2*q3*q3 + 1)* 57.3;
    Gyroscope_attitude_Angle_data_get_t->pitch  = asinf(-2*q1*q3 + 2*q0*q2)* 57.3;
    Gyroscope_attitude_Angle_data_get_t->roll = atan2f(2*(q2*q3 + q0*q1), -2*q1*q1 - 2*q2*q2 + 1)* 57.3;
}
