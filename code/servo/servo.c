/*
 * servo.c
 *
 *  Created on: 2023年3月24日
 *      Author: wzl
 */

#include "car_control/car_control.h"
#include "servo/servo.h"

//定义舵机运动矫正结构体参数
ServoStruct servoStr;


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机初始化
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void servo_init(void)
{
    //舵机初始化
    pwm_init(servo_pwm_out_pin, 250, (uint32)(medium_point * TO_PENCENT));    //占空比在0~10000.周期T=1/f=4ms
    //初始时，舵机在中点位置
    pwm_set_duty(servo_pwm_out_pin, (uint32)(medium_point * TO_PENCENT));
    //初始化线程相关参数
    servoStr.count = 0;
    servoStr.count_handle = false;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角度转换为pwm
// 参数说明     float     angle(-MAX_ANGLE ~ MAX_ANGLE)
// 返回参数     int16_t
// 说明        将90°设置为0点，最大正反转MAX_ANGLE
//-------------------------------------------------------------------------------------------------------------------
static int16_t servo_angel_to_pwm(float angle)
{
    //定义静态变量进行存储pwm值
    static float servo_pwm_in = 0;
    static float servo_pwm_out = 0;

    //由于车身限制的物理角度限幅
    if(angle >= MAX_ANGLE)
    {
        angle = MAX_ANGLE;
    }
    else if(angle <= -MAX_ANGLE)
    {
        angle = -MAX_ANGLE;
    }

    //将角度换算为对应的pwm-90°~90°对应-1000~1000
    servo_pwm_in = angle * translate_angle_to_pwm;
    //pwm偏移到500~2500
    servo_pwm_out = (float)medium_point + servo_pwm_in;
    //变量转换为int型，并调整是输出方向
    int16_t servo_pwm_out_int = 3000 - (int16_t)servo_pwm_out;

    //返回转换的pwm值
    return servo_pwm_out_int;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机控制，用于蓝牙调试
// 参数说明     float     angle(-MAX_ANGLE ~ MAX_ANGLE)
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void servo_contral(float angle)
{
    //定义存储变量
    static int16_t pwm_out;
    //舵机角度转换
    pwm_out = servo_angel_to_pwm(angle * 3.6);

    //舵机输出pwm限幅
    if(pwm_out >= SERVO_PWM_MAX)
    {
        pwm_out = SERVO_PWM_MAX;
    }
    else if(pwm_out <= SERVO_PWM_MIN)
    {
        pwm_out = SERVO_PWM_MIN;
    }

    //输出pwm控制角度
    pwm_set_duty(servo_pwm_out_pin, (uint32)(pwm_out * TO_PENCENT));
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机输出PWM设置（结合上位机进行矫正）
// 参数说明     uint16     pwm：500~2500
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void SERVO_SetPwmValueCorrect(uint16 pwm)
{
    //定义变量，处理传入的pwm值
    static uint16 pwm_Servo = 0;
    //调整输出方向
    pwm_Servo = 3000 - pwm;
    //进行中值补偿->pwm_Servo + (调整方向后的矫正中值 - 舵机实际中值)
    pwm_Servo = pwm_Servo + ((3000 - servoStr.thresholdMiddle) - medium_point);

    //输出pwm最值矫正
    uint16_t pwmMax = 3000 - servoStr.thresholdLeft;
    uint16_t pwmMin = 3000 - servoStr.thresholdRight;

    //由于物理的角度限制，防止损坏，软件进行限幅
    if(pwm_Servo < pwmMin)
        pwm_Servo = pwmMin;
    else if(pwm_Servo > pwmMax)
        pwm_Servo = pwmMax;

    //输出pwm控制角度
    pwm_set_duty(servo_pwm_out_pin, (uint32)(pwm_Servo * TO_PENCENT));
}
