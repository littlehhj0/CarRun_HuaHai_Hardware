#include "zf_common_headfile.h"
#include "isr_config.h"
#pragma section all "cpu0_dsram"


//包含头文件
#include "icm20602_data_handle.h"
#include "my_flash/my_flash.h"
#include "Buzzer/buzzer.h"
#include "Kalman_Filter.h"
#include "motor/motor.h"
#include "servo/servo.h"
#include "timer/timer.h"
#include "car_control.h"
#include "INA226.h"
#include "uart.h"
#include "pid.h"
/*
 * 初版测试方案
 * 电机测试
 * 舵机测试
 * 蜂鸣器测试
 * 无线串口测试
 *
 * 加入积分值打印 确定积分分离阈值大小
 * 加入PID模式打印 确定是否正确使用PID增量式
 *
 * 加入曲线绘制协议实现曲线绘制
 * 改进方案
 *
 *
 * 改良串口的通信速率，提升舵机响应速度！
 * 接收数据底层具体位置：uart_handle->asclin
 *
 *
 * PID相关参数做一个追踪，尤其是对于不同的PID用处做追踪 即底层电机PID与转向环PID追踪
 * 相关全局变量    PidTypeDef car_speed_pid
 *             IcarStruct icarStr;
 *             motor_ControlLoop
 *             ServoStruct servoStr;
 * 定时器中断     ccu60_ch1_timer_interrupt
 *             ccu61_ch0_timer_interrupt
 *
 * UART具体通信做一个追踪
 * 数据处理 USB_Edgeboard_Handle
 *
 *
 * 增加蜂鸣器测试装置
 * 增加无线串口直接断电指令(上位机已有)
 * 改进PID算法：增量式  积分分离（先追踪）
 * 增加陀螺仪配合
 * 增加FREERTOS(大概率不用)
 *
 *
 * 更新日志：
 * 5.4
 * 去掉flash的循环，将PID的赋值放在MOTOR的计算前面，同时将flashPIDEnable用作更新标志位，
 * 但是对于变量同时调用是否有互斥锁仍然不确认，要用串口测试
 * 增加了上位机增量控制模式切换 使用 USB_ADDR_SPEEDMODE 切换 请注意上位机原有通讯里没有该切换代码，具体参数需要调
 * 5.5
 * 重新回档了一版，因为PID不需要经常更改，增加了一个PID函数，改了一下FLASH与PID的顺序
 * 改进了增量式控制算法，增加了积分限幅与积分分离,在uart里的模式切换中增加了积分分离阈值的赋值协议,同时允许写入FLASH分离阈值与模式
 * 可以改进PID计算时间，具体值需要调试 Counter_speed_loop_set   在motor_ControlLoop内
 * 是否由上位机控制积分阈值，关闭时可以通过设置下位机初始值控制分离值 EDGE_CTRL
 *                 if(icarStr.SpeedSet >= 10)根据积分数据分析知
 *                 PD_USE
 * 下位机PID模式设置 my_flash #define PID_MODE PID_POSITION//宏定义设定模式，用于修改
 *    "Kp_speed": 12000,
    "Ki_speed": 250,
    "Kd_speed": 0,
        "speedLow": 1.3,
    "speedHigh": 2.3,
    "speedAI": 1.4,
    "speedRing": 1.4,
    "speedLow": 1.8,
    "speedHigh": 2.6,
    "speedAI": 1.8,
    "speedRing": 1.8,

        "speedLow": 1.9,
    "speedHigh": 2.7,
    "speedAI": 1.9,
    "speedRing": 1.8,


            "speedLow": 2.0,
    "speedHigh": 2.8,
    "speedAI": 2.0,
    "speedRing": 2.0,

    "Kp_speed": 14000,
    "Ki_speed": 250,
    "Kd_speed": 10,
    该参数极限

        "Kp_speed": 14000,
    "Ki_speed": 250,
    "Kd_speed": 15,
        "speedLow": 2.0,
    "speedHigh": 2.9,
    "speedAI": 2.0,
    "speedRing": 2.0,
    极限
           "speedLow": 2.0,
    "speedHigh": 3.0,
    "speedAI": 2.0,
    "speedRing": 2.0,
    最极限值：
     "speedLow": 2.0,
    "speedHigh": 3.1,
    "speedAI": 2.0,
    "speedRing": 2.0,
 * */

//----------------------------------代码区域----------------------------------
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口

//----------------------------------此处编写用户代码 例如外设初始化代码等----------------------------------
//    car_speed_pid.separationThreshold = 10000;
    //加载flash区的存储数据
    my_flash_init();
    //先初始化舵机，解决在中断中控制舵机卡死的bug
    servo_init();
    //电机初始化
    motor_init();
    //初始化电流采样
    INA226_Init();
    //定时器初始化
    timer_Init();
    //蜂鸣器初始化
    Buzzer_Init();
    //陀螺仪任务初始化
    icm20602_pose_init();
    //偏航角滤波，卡尔曼参数初始化
    Kalman_Filter_Init(&kalman_imu_yaw);
    //无线串口或蓝牙调试接口初始化
    Wireless_uart_init(wireless_using_uart, wireless_using_uart_baud, uart_wireless_pin_tx, uart_wireless_pin_rx);
//    Bluetooth_uart_init(bluetooth_using_uart, bluetooth_using_uart_baud, uart_booluteeth_pin_tx, uart_booluteeth_pin_rx);
    //通信连接提示灯初始化
    gpio_init(P20_9, GPO, 1, GPO_PUSH_PULL);
    //初始化完成后，拉高电平唤醒电机驱动板
    gpio_init(P21_3, GPO, 1, GPO_PUSH_PULL);
    //初始化引脚，用于提示数据接受是否错误
    gpio_init(P21_5, GPO, 1, GPO_PUSH_PULL);
    //卡尔曼参数初始化，电机返回速度滤波
    Kalman_Filter_Init(&kalman_motor_speedback);
    //智能车控制参数初始化
    ICAR_Init();
    //初始化完成，蜂鸣器提示音
    Buzzer_Enable(BuzzerSysStart);
//    PID_separationThreshold(&car_speed_pid,3);
//----------------------------------此处编写用户代码 例如外设初始化代码等----------------------------------

    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    while (TRUE)
    {

//----------------------------------此处编写需要循环执行的代码----------------------------------

        //flash任务处理
        my_flash_Handle();
        //蜂鸣器控制
        Buzzer_Handle();
        //智能车控制
        ICAR_Handle();
        //无线串口或蓝牙发送数据处理函数
        Wireless_Handle();
//        //蓝牙接受数据处理函数
//        use_bluetooth_Handle();
        //姿态角解算处理函数
        icm20602_attitude_Angle_handle();

//----------------------------------此处编写需要循环执行的代码----------------------------------

    }
}

#pragma section all restore
