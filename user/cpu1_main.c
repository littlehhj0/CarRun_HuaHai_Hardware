#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"


//����ͷ�ļ�
#include "voltage_sampling/voltage_sampling.h"
#include "zf_driver_pit.h"
#include "timer/timer.h"
#include "motor/motor.h"
#include "key/key.h"
#include "INA226.h"
#include "uart.h"


//-----------------------------------��������-----------------------------------
void core1_main(void)
{
    disable_Watchdog();                     // �رտ��Ź�
    interrupt_global_enable(0);             // ��ȫ���ж�

//-----------------------------------�˴���д�û����� ���������ʼ�������----------------------------------

    //��ѹ������ʼ��
    adc_Init();
    //������ʼ��
    my_key_init();
    //��ebͨ�ų�ʼ��
    USB_uart_init(eb_using_uart, eb_using_uart_baud, uart_eb_pin_tx, uart_eb_pin_rx);

//-----------------------------------�˴���д�û����� ���������ʼ�������-----------------------------------

    cpu_wait_event_ready();                 // �ȴ����к��ĳ�ʼ�����
    while (TRUE)
    {

//-----------------------------------�˴���д��Ҫѭ��ִ�еĴ���-----------------------------------

        //��ѹ��������
        adc_Handle();
        //���ݴ���
        USB_Edgeboard_Handle();

//-----------------------------------�˴���д��Ҫѭ��ִ�еĴ���-----------------------------------

    }
}

#pragma section all restore
//----------------------------------��������----------------------------------
