/*
 * motor.c
 *
 *  Created on: 2023��3��20��
 *      Author: wzl
 */

#include "SysSe/Math/Ifx_LowPassPt1F32.h"
#include "Kalman_Filter.h"
#include "car_control.h"
#include "motor.h"
#include "uart.h"
#include "pid.h"
#include "my_flash.h"

#define Counter_speed_loop_set 10 //���ü����� ��λ0.5ms
//�������˶��ṹ�����
MotorStruct motorStr;
// ����һ�׵�ͨ�˲����ṹ��
Ifx_LowPassPt1F32 LowPass_filter_current;
// ����һ�׵�ͨ�˲����Ĳ������ýṹ��
Ifx_LowPassPt1F32_Config LowPass_filter_config_current;
// ����һ�׵�ͨ�˲����ṹ��
Ifx_LowPassPt1F32 LowPass_filter_speedset;
// ����һ�׵�ͨ�˲����Ĳ������ýṹ��
Ifx_LowPassPt1F32_Config LowPass_filter_config_speedset;


//-------------------------------------------------------------------------------------------------------------------
// �������     �����ʼ��
// ����˵��     void
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void motor_init(void)
{
    //�ٶȿ��Ƴ�ʼ��
    pwm_init(pwm_rotation, 50000, 0);                        //50KHZ ռ�ձ���0~10000.
    //����ת���Ƴ�ʼ��
    gpio_init(pwm_control, GPO, 0, GPO_PUSH_PULL);           //���Ƶ�����򣬳�ʼ�͵�ƽ

    //��ʼʱ������ٶ�Ϊ0
    motor_SetPwmValue(0);

    //��������ʼ��
    encoder_quad_init(USING_TIMER, encoder_pin_CH1, encoder_pin_CH2);
    //���������������
    encoder_clear_count(USING_TIMER);//    MODULE_GPT120->T2.U = 0;

    //���ģ�ͳ�ʼ��
    motorStr.EncoderLine = 500.0f;                          //����������
    motorStr.ReductionRatio = 1.0f;                         //������ٱ�
    motorStr.EncoderValue = 0;                              //��ʼ��������ʵʱ�ٶ�
    motorStr.DiameterWheel = 0.062f;                        //����ֱ��62cm���ò�����λΪm
    motorStr.CloseLoop = 1;                                 //1:�ջ�ģʽ | 0:����ģʽ

    // ��ͨ�˲����ĳ�ʼ��
    LowPass_filter_config_current.cutOffFrequency = 10;                                      // ��ͨ�˲����Ľ�ֹƵ��Ϊ10hz
    LowPass_filter_config_current.samplingTime = 0.005;                                      // �˲������ݲ���ʱ��Ϊ0.005s
    LowPass_filter_config_current.gain = 0.85;                                               // ��ͨ�˲���������Ϊ0.85
    Ifx_LowPassPt1F32_init(&LowPass_filter_current, &LowPass_filter_config_current);         // ��ͨ�˲����ĳ�ʼ��

    // ��ͨ�˲����ĳ�ʼ��
    LowPass_filter_config_speedset.cutOffFrequency = 10;                                      // ��ͨ�˲����Ľ�ֹƵ��Ϊ10hz
    LowPass_filter_config_speedset.samplingTime = 0.005;                                      // �˲������ݲ���ʱ��Ϊ0.005s��200HZ
    LowPass_filter_config_speedset.gain = 0.95;                                               // ��ͨ�˲���������Ϊ0.95
    Ifx_LowPassPt1F32_init(&LowPass_filter_speedset, &LowPass_filter_config_speedset);        // ��ͨ�˲����ĳ�ʼ��
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ���ٶ�
// ����˵��     int16 pwm     �����ٶȺ�����ת ��Χ��-10000~10000
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void motor_SetPwmValue(int16 pwm)
{
    if(pwm >= 0)
    {
        //�޷�����ֹ������Χ
        if(pwm >= PWM_DUTY_MAX)
        {
            pwm = PWM_DUTY_MAX;
        }
        //���õ����ת
        gpio_set_level(pwm_control, 1);//���Ƶķ�������
        pwm_set_duty(pwm_rotation, (uint32)pwm);
    }
    else
    {
        //�����ű�Ϊ��
        pwm = -pwm;
        //�޷�����ֹ������Χ
        if(pwm >= PWM_DUTY_MAX)
        {
            pwm = PWM_DUTY_MAX;
        }
        //���õ����ת
        gpio_set_level(pwm_control, 0);
        pwm_set_duty(pwm_rotation, (uint32)pwm);
    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ����ٶȻ�+������
// ����˵��     void
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void motor_ControlLoop(void)
{
    // �������pwmֵ
    static int16 speed_to_pwm = 0;

    // �߳̿��ƣ�ÿ0.5ms����һ�θú���
    motorStr.Counter_speed_loop++;
//    motorStr.Counter_current_loop++;

//    // ������������ƣ�ÿ1ms����һ��
//    if(motorStr.Counter_current_loop >= 2)
//    {
//        // ͨ�����Ӳűջ�
//        if(usbStr.connected && motorStr.CloseLoop)
//        {
//            // ���������pid����������
////            motorStr.Set_current_motor = icarStr.SpeedSet;
//            PID_Calc(&car_current_pid, motorStr.Current_motor_After_filter, motorStr.Set_current_motor);
//
//            // ��ֵpwm
//            speed_to_pwm = (int16)(car_current_pid.out);
//            motor_SetPwmValue(speed_to_pwm);
//        }
//
//        // ����߳�
//        motorStr.Counter_current_loop = 0;
//    }

    // ÿ5ms�Ե���ٶȻ����п���
    if(motorStr.Counter_speed_loop >= Counter_speed_loop_set)
    {
        // ��ȡ��ǰ��������ֵ
        motorStr.EncoderValue = encoder_get_count(USING_TIMER);
        // ��ն�ʱ����ֵ
        encoder_clear_count(USING_TIMER);
        // ��ȡʵ���ٶȣ����㹫ʽ����ʱ������ֵ/4��Ƶ/����������/����ļ��ٱ�/ѭ��ʱ��*���Ӱ뾶*PI
        float temp_speed = (float)(motorStr.EncoderValue * motorStr.DiameterWheel * PI_MOTOR)/ MOTOR_CONTROL_CYCLE / motorStr.EncoderLine / 4.0f / motorStr.ReductionRatio;
        // ���ڷ����ٶȽ��п������˲�
        icarStr.SpeedFeedback = Kalman_Filter_Fun(&kalman_motor_speedback, temp_speed);
//        icarStr.SpeedFeedback = temp_speed;

        //ͨ�����Ӳűջ�
        if(usbStr.connected)
        {
            //�ջ��ٿ�
            if(motorStr.CloseLoop)
            {
//                /*  5.4 ÿ�μ���ǰ����    */
//                    if(flashPIDEnable)//flashPIDEnable�䵱���±�־λ
//                    {
//                            // ��ʼ��speed_loop_pid����
//                            float speed_loop_pid_buff[3] = {icarStr.speed_loop_Kp, icarStr.speed_loop_Ki, icarStr.speed_loop_Kd};
//                            PID_Init(&car_speed_pid, PID_POSITION, speed_loop_pid_buff, SPEED_LOOP_MAX_OUT, SPEED_LOOP_P_MAX_OUT);
//                            // ��ʼ��current_loop_pid����
//                            float current_loop_pid_buff[3] = {icarStr.current_loop_Kp, icarStr.current_loop_Ki, icarStr.current_loop_Kd};
//                            PID_Init(&car_current_pid, PID_POSITION, current_loop_pid_buff, CURRENT_LOOP_MAX_OUT, CURRENT_LOOP_P_MAX_OUT);
//                            // д����ɣ��ر�д��״̬
//                            flashPIDEnable = false;
//                    }
                // �ٶȻ�pid����������
                icarStr.speed_set = Ifx_LowPassPt1F32_do(&LowPass_filter_speedset, icarStr.SpeedSet);
                PID_Calc(&car_speed_pid, icarStr.SpeedFeedback, icarStr.SpeedSet);
                // ������pid��������������
//                motorStr.Set_current_motor = Ifx_LowPassPt1F32_do(&LowPass_filter_current, car_speed_pid.out);
//                if(icarStr.SpeedSet >= 0)
//                    motorStr.Set_current_motor = car_speed_pid.out + 0.15;
//                else
//                    motorStr.Set_current_motor = car_speed_pid.out - 0.15;
//                motorStr.Set_current_motor = car_speed_pid.out;

                // ��ֵpwm
                speed_to_pwm = (int16)(car_speed_pid.out);
                motor_SetPwmValue(speed_to_pwm);
            }
            else
            {
                // �����ٷֱȿ���
                if(icarStr.SpeedSet >= 10)
                    icarStr.SpeedSet = 10;
                else if(icarStr.SpeedSet <= -10)
                    icarStr.SpeedSet = -10;
                // �������ٷֱȣ������ٶ���ϢΪ0-10��10Ϊ��ת
                speed_to_pwm = (int16)(SPEED_LOOP_MAX_OUT * icarStr.SpeedSet / 10.0f);
                // ��ֵpwm
                motor_SetPwmValue(speed_to_pwm);
            }
        }
        else
        {
            // ͨ��δ���ӣ������pwm
            motor_SetPwmValue(0);
        }

        // ����߳�
        motorStr.Counter_speed_loop = 0;
    }
}
