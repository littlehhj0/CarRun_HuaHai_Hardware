/*
 * servo.h
 *
 *  Created on: 2023��3��24��
 *      Author: wzl
 */

#ifndef CODE_SERVO_SERVO_H_
#define CODE_SERVO_SERVO_H_

//����ͷ�ļ�
#include "zf_driver_pwm.h"
#include "cpu0_main.h"

//�궨������������
#define servo_pwm_out_pin   13      //ʹ�õ�ATOM0_CH1_P33_9

//��������     ���ָ��ʱ�任��Ϊռ�ձ�
#define TO_PENCENT          (2.5f)
//�궨�����PWM��Сֵ
#define SERVO_PWM_MIN       500
//�궨�����PWM���ֵ
#define SERVO_PWM_MAX       2500
//�궨��pwm��ֵ  500US-2500US
#define medium_point        1500
//�궨���������������ת���Ƕ�
#define MAX_ANGLE           38.0f
//�궨��ǶȻ���pwm��ϵʽ
#define translate_angle_to_pwm  (1000.0f / 90.0f)

//�����������װ������Ҫ��������λ�����Ͳ��������в����������������½ṹ��
typedef struct
{
    uint16_t thresholdMiddle;                   //�����ֵPWM
    uint16_t thresholdLeft;                     //�������ת�����ֵPWM
    uint16_t thresholdRight;                    //�������ת�����ֵPWM
    uint8_t  count;                             //�̼߳�����
    bool     count_handle;                      //�̴߳����־λ
}ServoStruct;

//��������˶������ṹ�����
extern ServoStruct servoStr;

//====================================================SERVO ��������====================================================
void servo_init                 (void);
void servo_contral              (float angle);
void SERVO_SetPwmValueCorrect   (uint16 pwm);
void SERVO_Timer                (void);
void SERVO_Handle               (void);
//====================================================SERVO ��������====================================================

#endif /* CODE_SERVO_SERVO_H_ */
