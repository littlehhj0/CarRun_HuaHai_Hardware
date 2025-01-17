/*
 * car_control.c
 *
 *  Created on: 2023年3月27日
 *      Author: wzl
 */

#include "voltage_sampling/voltage_sampling.h"
#include "car_control/car_control.h"
#include "motor/motor.h"
#include "servo/servo.h"
#include "uart/uart.h"
#include "key/key.h"

//定义小车控制参数到的结构体  全局变量控制核心！
IcarStruct icarStr;


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     智能车参数初始化
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void ICAR_Init(void)
{
    icarStr.Voltage = 0;                             //电压
    icarStr.SpeedSet = 0.0f;                         //电机目标速度：m/s
    icarStr.SpeedFeedback = 0.0f;                    //电机模型实测速度：m/s
    icarStr.ServoPwmSet = servoStr.thresholdMiddle;  //设置舵机初始化pwm
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     智能车综合处理线程计数器
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void ICAR_Timer(void)
{
    //如果检测到按键按下，开始计数本次按键按下的时长，直到松开
    if(gpioStr.KeyPress)
    {
        icarStr.counterKeyA++;
    }
    else
    {
        //松开按键后，如果本次按键按下时长大于100ms，并改变小车控制参数的按键状态，有按键按下过，根据按键的时长，来进行相关的处理
        if(icarStr.counterKeyA > 100)
            icarStr.keyPressed = true;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     智能车综合处理函数
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void ICAR_Handle(void)
{
    //根据线程检测，有按键按下并且时长大于100ms，进行相关的处理操作
    if(icarStr.keyPressed)
    {
        //再次确认按键时长，防止出错
        if(icarStr.counterKeyA > 100)
            //向上位机发送按键时长，通知上位机可以进行发车
#if !USING_BLUETOOTH_OR_EGBOARD
            USB_Edgeboard_TransmitKey(icarStr.counterKeyA);
#endif

        //如果按键按下时长超过3秒，进行相关操作
        if(icarStr.counterKeyA > 3000 && !icarStr.selfcheckEnable)
        {
            // 是否采用电量提醒设置
            if(adc_sampling.key_contral == true)
            {
                adc_sampling.key_contral = false;
            }
            else
            {
                adc_sampling.key_contral = true;
            }
        }

        //完成按键信息处理后，清除按键相关信息
        icarStr.counterKeyA = 0;
        icarStr.keyPressed = false;
        //如果开启自检功能，测试按键好坏
        if(icarStr.selfcheckEnable)
        {
            icarStr.errorCode &= ~(1<<8);
        }
    }

    //智能车自检功能
    if(icarStr.selfcheckEnable)
    {
        //ICAR_Selfcheck();
    }
}
