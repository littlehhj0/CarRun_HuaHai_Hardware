#include "zf_common_headfile.h"
#include "isr_config.h"
#pragma section all "cpu0_dsram"


//����ͷ�ļ�
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
 * ������Է���
 * �������
 * �������
 * ����������
 * ���ߴ��ڲ���
 *
 * �������ֵ��ӡ ȷ�����ַ�����ֵ��С
 * ����PIDģʽ��ӡ ȷ���Ƿ���ȷʹ��PID����ʽ
 *
 * �������߻���Э��ʵ�����߻���
 * �Ľ�����
 *
 *
 * �������ڵ�ͨ�����ʣ����������Ӧ�ٶȣ�
 * �������ݵײ����λ�ã�uart_handle->asclin
 *
 *
 * PID��ز�����һ��׷�٣������Ƕ��ڲ�ͬ��PID�ô���׷�� ���ײ���PID��ת��PID׷��
 * ���ȫ�ֱ���    PidTypeDef car_speed_pid
 *             IcarStruct icarStr;
 *             motor_ControlLoop
 *             ServoStruct servoStr;
 * ��ʱ���ж�     ccu60_ch1_timer_interrupt
 *             ccu61_ch0_timer_interrupt
 *
 * UART����ͨ����һ��׷��
 * ���ݴ��� USB_Edgeboard_Handle
 *
 *
 * ���ӷ���������װ��
 * �������ߴ���ֱ�Ӷϵ�ָ��(��λ������)
 * �Ľ�PID�㷨������ʽ  ���ַ��루��׷�٣�
 * �������������
 * ����FREERTOS(����ʲ���)
 *
 *
 * ������־��
 * 5.4
 * ȥ��flash��ѭ������PID�ĸ�ֵ����MOTOR�ļ���ǰ�棬ͬʱ��flashPIDEnable�������±�־λ��
 * ���Ƕ��ڱ���ͬʱ�����Ƿ��л�������Ȼ��ȷ�ϣ�Ҫ�ô��ڲ���
 * ��������λ����������ģʽ�л� ʹ�� USB_ADDR_SPEEDMODE �л� ��ע����λ��ԭ��ͨѶ��û�и��л����룬���������Ҫ��
 * 5.5
 * ���»ص���һ�棬��ΪPID����Ҫ�������ģ�������һ��PID����������һ��FLASH��PID��˳��
 * �Ľ�������ʽ�����㷨�������˻����޷�����ַ���,��uart���ģʽ�л��������˻��ַ�����ֵ�ĸ�ֵЭ��,ͬʱ����д��FLASH������ֵ��ģʽ
 * ���ԸĽ�PID����ʱ�䣬����ֵ��Ҫ���� Counter_speed_loop_set   ��motor_ControlLoop��
 * �Ƿ�����λ�����ƻ�����ֵ���ر�ʱ����ͨ��������λ����ʼֵ���Ʒ���ֵ EDGE_CTRL
 *                 if(icarStr.SpeedSet >= 10)���ݻ������ݷ���֪
 *                 PD_USE
 * ��λ��PIDģʽ���� my_flash #define PID_MODE PID_POSITION//�궨���趨ģʽ�������޸�
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
    �ò�������

        "Kp_speed": 14000,
    "Ki_speed": 250,
    "Kd_speed": 15,
        "speedLow": 2.0,
    "speedHigh": 2.9,
    "speedAI": 2.0,
    "speedRing": 2.0,
    ����
           "speedLow": 2.0,
    "speedHigh": 3.0,
    "speedAI": 2.0,
    "speedRing": 2.0,
    ���ֵ��
     "speedLow": 2.0,
    "speedHigh": 3.1,
    "speedAI": 2.0,
    "speedRing": 2.0,
 * */

//----------------------------------��������----------------------------------
int core0_main(void)
{
    clock_init();                   // ��ȡʱ��Ƶ��<��ر���>
    debug_init();                   // ��ʼ��Ĭ�ϵ��Դ���

//----------------------------------�˴���д�û����� ���������ʼ�������----------------------------------
//    car_speed_pid.separationThreshold = 10000;
    //����flash���Ĵ洢����
    my_flash_init();
    //�ȳ�ʼ�������������ж��п��ƶ��������bug
    servo_init();
    //�����ʼ��
    motor_init();
    //��ʼ����������
    INA226_Init();
    //��ʱ����ʼ��
    timer_Init();
    //��������ʼ��
    Buzzer_Init();
    //�����������ʼ��
    icm20602_pose_init();
    //ƫ�����˲���������������ʼ��
    Kalman_Filter_Init(&kalman_imu_yaw);
    //���ߴ��ڻ��������Խӿڳ�ʼ��
    Wireless_uart_init(wireless_using_uart, wireless_using_uart_baud, uart_wireless_pin_tx, uart_wireless_pin_rx);
//    Bluetooth_uart_init(bluetooth_using_uart, bluetooth_using_uart_baud, uart_booluteeth_pin_tx, uart_booluteeth_pin_rx);
    //ͨ��������ʾ�Ƴ�ʼ��
    gpio_init(P20_9, GPO, 1, GPO_PUSH_PULL);
    //��ʼ����ɺ����ߵ�ƽ���ѵ��������
    gpio_init(P21_3, GPO, 1, GPO_PUSH_PULL);
    //��ʼ�����ţ�������ʾ���ݽ����Ƿ����
    gpio_init(P21_5, GPO, 1, GPO_PUSH_PULL);
    //������������ʼ������������ٶ��˲�
    Kalman_Filter_Init(&kalman_motor_speedback);
    //���ܳ����Ʋ�����ʼ��
    ICAR_Init();
    //��ʼ����ɣ���������ʾ��
    Buzzer_Enable(BuzzerSysStart);
//    PID_separationThreshold(&car_speed_pid,3);
//----------------------------------�˴���д�û����� ���������ʼ�������----------------------------------

    cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����
    while (TRUE)
    {

//----------------------------------�˴���д��Ҫѭ��ִ�еĴ���----------------------------------

        //flash������
        my_flash_Handle();
        //����������
        Buzzer_Handle();
        //���ܳ�����
        ICAR_Handle();
        //���ߴ��ڻ������������ݴ�����
        Wireless_Handle();
//        //�����������ݴ�����
//        use_bluetooth_Handle();
        //��̬�ǽ��㴦����
        icm20602_attitude_Angle_handle();

//----------------------------------�˴���д��Ҫѭ��ִ�еĴ���----------------------------------

    }
}

#pragma section all restore
