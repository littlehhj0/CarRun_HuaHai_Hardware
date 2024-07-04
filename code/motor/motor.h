/*
 * motor.h
 *
 *  Created on: 2023��3��20��
 *      Author: wzl
 */

#ifndef CODE_MOTOR_MOTOR_H_
#define CODE_MOTOR_MOTOR_H_

//����ͷ�ļ�
#include "cpu0_main.h"
#include "uart/uart.h"


//���pwm������ź궨��
#define pwm_rotation            70        //ʹ�õ�ATOM1_CH0_P21_4������ת�٣�
#define pwm_control             674       //ʹ�õ�IO���ſ��Ʒ���21_2


//�������������ź궨��
#define encoder_pin_CH1         1         //33.7   TIM2_ENCODER_CH1_P33_7
#define encoder_pin_CH2         1         //33.6
//ʹ�ö�ʱ���궨��
#define USING_TIMER             0         //TIM2
/*ȡ��ԭ�� �ڸ�ö��������0��TIM2
 * typedef enum  // ö�� ��ʱ�����
{
    TIM2_ENCODER,
    TIM3_ENCODER,
    TIM4_ENCODER,
    TIM5_ENCODER,
    TIM6_ENCODER,
}encoder_index_enum;
*/

//�������ٶȻ�������������޷��궨��
//#define SPEED_LOOP_MAX_OUT       5
//#define SPEED_LOOP_P_MAX_OUT     1
#define CURRENT_LOOP_MAX_OUT     10000
#define CURRENT_LOOP_P_MAX_OUT   8000
#define SPEED_LOOP_MAX_OUT       10000
#define SPEED_LOOP_P_MAX_OUT     5000
#define SPEED_LOOP_I_MAX_OUT     30


//�궨��pi
#define PI_MOTOR                 3.1415926535898f
//����ٶȻ���������Ϊ5ms
#define MOTOR_CONTROL_CYCLE      0.005f


//�����������ṹ��
typedef struct
{
    float    ReductionRatio;                    //������ٱ�
    float    EncoderLine;                       //����������
    int16    EncoderValue;                      //������ʵʱ�ٶ�,��ʱ����ֵ
    float    DiameterWheel;                     //����ֱ����mm
    uint8_t  CloseLoop;                         //����ģʽ

    uint16_t Counter_speed_loop;                //�ٶȻ��̼߳�����
    uint16_t Counter_current_loop;              //�������̼߳�����

    float Current_motor;                        //�����������
    float Current_motor_After_filter;           //��ͨ�˲���ĵ���ֵ
    float Set_current_motor;                    //����趨Ŀ�����
}MotorStruct;


//������������ṹ��
extern MotorStruct motorStr;


//====================================================MOTOR ��������====================================================
void motor_init                 (void);
void motor_SetPwmValue          (int16_t pwm);
void motor_ControlLoop          (void);
//====================================================MOTOR ��������====================================================

#endif
