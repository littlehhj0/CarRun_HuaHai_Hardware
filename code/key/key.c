/*
 * key.c
 *
 *  Created on: 2023年3月28日
 *      Author: wzl
 */

#include "Buzzer/buzzer.h"
#include "isr_config.h"
#include "key/key.h"

//定义按键结构体
GpioStruct gpioStr;


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       按键初始化
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void my_key_init(void)
{
    //初始化引脚
    gpio_init(KEY_GPIO, GPI, GPIO_HIGH, GPI_PULL_UP);
    //外部引脚中断初始化,p15_4,双边沿促发中断
    exti_init(ERU_CH0_REQ0_P15_4, EXTI_TRIGGER_BOTH);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       按键中断响应函数
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void exti_interrupt(void)
{
    //按键按下,为低电平
    if(!gpio_get_level(KEY_GPIO))
    {
        //改变按键的状态
        gpioStr.KeyPress = true;
        //蜂鸣器提示
        Buzzer_Enable(BuzzerDing);
    }
    else
    {
        //双边沿触发中断，按键弹起触发中断，且为高电平，改变按键状态为松开
        gpioStr.KeyPress = false;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       按键中断向量
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    // 开启中断嵌套
    interrupt_global_enable(0);
    // 通道0中断
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))
    {
        //清除中断标志位
        exti_flag_clear(ERU_CH0_REQ0_P15_4);
        //中断服务函数
        exti_interrupt();
    }
    // 通道4中断
    if(exti_flag_get(ERU_CH4_REQ13_P15_5))
    {
        //清除中断标志位
        exti_flag_clear(ERU_CH4_REQ13_P15_5);
    }
}
