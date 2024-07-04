/*
 * pid.h
 *
 *  Created on: 2023��3��18��
 *      Author: wzl
 */


#ifndef PID_H
#define PID_H


#include "cpu0_main.h"


// pid������ģʽö��
enum PID_MODE
{
    PID_POSITION = 0,//λ��ʽPID
    PID_DELTA        //����ʽPID
};


// pid�������ṹ��
typedef struct
{
    uint8_t mode_t;

    //PID ����
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //������
    float max_iout; //���������

    float set;      //�趨ֵ
    float fdb;      //ʵ��ֵ
    float separationThreshold;//���ַ�����ֵ����������ֵʱPD�����С����ֵʱPID

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�
} PidTypeDef;


//��������ٶȻ��ṹ��
extern PidTypeDef car_speed_pid;
//��������������ṹ��
extern PidTypeDef car_current_pid;
extern uint8 PD_USE;//���ƻ��ַ����Ƿ����� 1 ����


//====================================================PID ��������====================================================
extern void  PID_Init        (PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);
extern float PID_Calc        (PidTypeDef *pid, float ref, float set);
extern void  PID_clear       (PidTypeDef *pid);
extern void  PID_Change      (PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);
extern void  PID_separationThreshold(PidTypeDef *pid, float separationThreshold);
//====================================================PID ��������====================================================


#endif
