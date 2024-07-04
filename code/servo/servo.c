/*
 * servo.c
 *
 *  Created on: 2023��3��24��
 *      Author: wzl
 */

#include "car_control/car_control.h"
#include "servo/servo.h"

//�������˶������ṹ�����
ServoStruct servoStr;


//-------------------------------------------------------------------------------------------------------------------
// �������     �����ʼ��
// ����˵��     void
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void servo_init(void)
{
    //�����ʼ��
    pwm_init(servo_pwm_out_pin, 250, (uint32)(medium_point * TO_PENCENT));    //ռ�ձ���0~10000.����T=1/f=4ms
    //��ʼʱ��������е�λ��
    pwm_set_duty(servo_pwm_out_pin, (uint32)(medium_point * TO_PENCENT));
    //��ʼ���߳���ز���
    servoStr.count = 0;
    servoStr.count_handle = false;
}


//-------------------------------------------------------------------------------------------------------------------
// �������     �Ƕ�ת��Ϊpwm
// ����˵��     float     angle(-MAX_ANGLE ~ MAX_ANGLE)
// ���ز���     int16_t
// ˵��        ��90������Ϊ0�㣬�������תMAX_ANGLE
//-------------------------------------------------------------------------------------------------------------------
static int16_t servo_angel_to_pwm(float angle)
{
    //���徲̬�������д洢pwmֵ
    static float servo_pwm_in = 0;
    static float servo_pwm_out = 0;

    //���ڳ������Ƶ�����Ƕ��޷�
    if(angle >= MAX_ANGLE)
    {
        angle = MAX_ANGLE;
    }
    else if(angle <= -MAX_ANGLE)
    {
        angle = -MAX_ANGLE;
    }

    //���ǶȻ���Ϊ��Ӧ��pwm-90��~90���Ӧ-1000~1000
    servo_pwm_in = angle * translate_angle_to_pwm;
    //pwmƫ�Ƶ�500~2500
    servo_pwm_out = (float)medium_point + servo_pwm_in;
    //����ת��Ϊint�ͣ����������������
    int16_t servo_pwm_out_int = 3000 - (int16_t)servo_pwm_out;

    //����ת����pwmֵ
    return servo_pwm_out_int;
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ������ƣ�������������
// ����˵��     float     angle(-MAX_ANGLE ~ MAX_ANGLE)
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void servo_contral(float angle)
{
    //����洢����
    static int16_t pwm_out;
    //����Ƕ�ת��
    pwm_out = servo_angel_to_pwm(angle * 3.6);

    //������pwm�޷�
    if(pwm_out >= SERVO_PWM_MAX)
    {
        pwm_out = SERVO_PWM_MAX;
    }
    else if(pwm_out <= SERVO_PWM_MIN)
    {
        pwm_out = SERVO_PWM_MIN;
    }

    //���pwm���ƽǶ�
    pwm_set_duty(servo_pwm_out_pin, (uint32)(pwm_out * TO_PENCENT));
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ������PWM���ã������λ�����н�����
// ����˵��     uint16     pwm��500~2500
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void SERVO_SetPwmValueCorrect(uint16 pwm)
{
    //����������������pwmֵ
    static uint16 pwm_Servo = 0;
    //�����������
    pwm_Servo = 3000 - pwm;
    //������ֵ����->pwm_Servo + (���������Ľ�����ֵ - ���ʵ����ֵ)
    pwm_Servo = pwm_Servo + ((3000 - servoStr.thresholdMiddle) - medium_point);

    //���pwm��ֵ����
    uint16_t pwmMax = 3000 - servoStr.thresholdLeft;
    uint16_t pwmMin = 3000 - servoStr.thresholdRight;

    //��������ĽǶ����ƣ���ֹ�𻵣���������޷�
    if(pwm_Servo < pwmMin)
        pwm_Servo = pwmMin;
    else if(pwm_Servo > pwmMax)
        pwm_Servo = pwmMax;

    //���pwm���ƽǶ�
    pwm_set_duty(servo_pwm_out_pin, (uint32)(pwm_Servo * TO_PENCENT));
}
