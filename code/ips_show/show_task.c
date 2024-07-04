/*
 * show_task.c
 *
 *  Created on: 2023��3��18��
 *      Author: wzl
 */

//����ͷ�ļ�
#include "icm20602_data_handle.h"
#include "show_task.h"


//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʾ����ʼ��
// ����˵��     void
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void ips_show_init(void)
{
    //������ʾ����
    ips200_set_dir(IPS200_PORTAIT);
    //��������
    ips200_set_font(IPS200_8X16_FONT);
    //��ʾ��Ӳ����ʼ��
    ips200_init(IPS200_TYPE_PARALLEL8);
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʾ����
// ����˵��     void
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void ips_show_task(void)
{
    //��ʾyawֵ
    ips200_show_string(0, 0, "yaw:");
    ips200_show_float(32, 0, Gyroscope_attitude_Angle_data_get.yaw, 3, 2);
    //��ʾpitchֵ
    ips200_show_string(0, 20, "pitch:");
    ips200_show_float(48, 0, Gyroscope_attitude_Angle_data_get.pitch, 3, 2);
    //��ʾrollֵ
    ips200_show_string(0, 40, "roll:");
    ips200_show_float(40, 0, Gyroscope_attitude_Angle_data_get.roll, 3, 2);
}
