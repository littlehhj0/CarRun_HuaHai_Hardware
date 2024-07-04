/*
 * car_control.h
 *
 *  Created on: 2023��3��27��
 *      Author: wzl
 */

#ifndef CODE_CAR_CONTROL_CAR_CONTROL_H_
#define CODE_CAR_CONTROL_CAR_CONTROL_H_

//����ͷ�ļ�
#include "cpu0_main.h"


//���ܳ��Լ첽��ö�٣�δʹ�ù�
typedef enum
{
    Selfcheck_None = 0,             //��ʼ����
    Selfcheck_MotorA,               //�����ת����
    Selfcheck_MotorB,               //�����ת����
    Selfcheck_MotorC,               //�����ת����
    Selfcheck_MotorD,               //�����ת����
    Selfcheck_MotorE,               //����ջ���������
    Selfcheck_MotorF,               //����ջ���������
    Selfcheck_MotorG,               //����ջ���ת����
    Selfcheck_MotorH,               //����ջ���ת����
    Selfcheck_ServoA,               //�������A
    Selfcheck_Com,                  //ͨ�Ų���
    Selfcheck_Buzzer,               //����������
    Selfcheck_RgbLed,               //��Ч����
    Selfcheck_Key,                  //��������
    Selfcheck_Finish                //�������
}SelfcheckEnum;


//���ܳ���ز������ƽṹ��
typedef struct
{
    float  Voltage;                           //��ص�ѹ
    float  SpeedSet;                          //���Ŀ���ٶȣ�m/s
    float  speed_set;                         //�˲����Ŀ���ٶȣ�m/s
    float  SpeedFeedback;                     //���ģ��ʵ���ٶȣ�m/s
    uint16 ServoPwmSet;                       //���PWM����

    float  speed_loop_Kp;                     //�ٶȻ�kp
    float  speed_loop_Ki;                     //�ٶȻ�ki
    float  speed_loop_Kd;                     //�ٶȻ�kd
    float  current_loop_Kp;                   //������kp
    float  current_loop_Ki;                   //������ki
    float  current_loop_Kd;                   //������kd

    uint16 counterKeyA;                       //����ģʽA������
    bool   keyPressed;                        //��������
    uint16 errorCode;                         //�������

    bool   selfcheckEnable;                   //���ܳ��Լ�ʹ��
    uint16 counterSelfcheck;                  //�Լ������
    uint8  timesSendStep;                     //���ͳ�ʱ���ݴ���
    uint16 counterModuleCheck;                //�Լ������
    SelfcheckEnum selfcheckStep;              //�Լ첽��
    uint8  speedSampleStep;                   //�ٶȲ�������
}IcarStruct;


//�����ṹ��
extern IcarStruct icarStr;


//==================================================CAR_CONTROL ��������==================================================
void ICAR_Init          (void);
void ICAR_Timer         (void);
void ICAR_Handle        (void);
//==================================================CAR_CONTROL ��������==================================================


#endif
