/*
 * uart.c
 *
 *  Created on: 2023��3��18��
 *      Author: wzl
 */


//����ͷ�ļ�
#include "uart.h"
#include "pid/pid.h"
#include "isr_config.h"
#include "timer/timer.h"
#include "servo/servo.h"
#include "motor/motor.h"
#include "Buzzer/buzzer.h"
#include "my_flash/my_flash.h"
#include "icm20602_data_handle.h"
#include "car_control/car_control.h"


//�����������ݽṹ��
BlueTooth_data_recevie_Struct Bluetooth_data;
//�������ߴ������ݽṹ��
Wireless_data_recevie_Struct Wireless_data;
//����ebͨ�����ݽṹ��
UsbStruct usbStr;


//-------------------------------------------------------------------------------------------------------------------
// �������     ��������λ��ͨ�ŵĴ��ڳ�ʼ��
// ����˵��     uartn           ����ģ���(UART_0,UART_1,UART_2,UART_3)
// ����˵��     baud            ���ڲ�����
// ����˵��     tx_pin          ���ڷ�������
// ����˵��     rx_pin          ���ڽ�������
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void USB_uart_init(uart_index_enum uartn, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin)
{
    // ���ڳ�ʼ��
    uart_init(uartn, baud, tx_pin, rx_pin);

    // ���������ж�
    uart_tx_interrupt(uartn, 1);
    uart_rx_interrupt(uartn, 1);

    // UsbStruct���ݳ�ʼ��
    usbStr.counter = 0;                         //����ʱ���������ʼ��Ϊ0
    usbStr.receiveFinished = false;
    usbStr.receiveStart = false;
    usbStr.receiveIndex = 0;
    usbStr.connected = false;                   //��ʼ��Ĭ��δ����λ������
    usbStr.inspectorEnable = false;
    usbStr.counterSend_upper = 0;               //��ʼ�������ٶȵļ�ʱ��
}


//-------------------------------------------------------------------------------------------------------------------
// �������     �������ߴ��ڵ��ԵĴ��ڳ�ʼ��
// ����˵��     uartn           ����ģ���(UART_0,UART_1,UART_2,UART_3)
// ����˵��     baud            ���ڲ�����
// ����˵��     tx_pin          ���ڷ�������
// ����˵��     rx_pin          ���ڽ�������
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void Wireless_uart_init(uart_index_enum uartn, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin)
{
    // ���ڳ�ʼ��
    uart_init(uartn, baud, tx_pin, rx_pin);

    // Bluetooth_data���ݳ�ʼ��
    Wireless_data.Flag_Wireless = false;       //���ߴ��ڶ�ʱ���ͱ�־λ
}


//-------------------------------------------------------------------------------------------------------------------
// �������     �������ߴ��ڵ��ԵĴ��ڳ�ʼ��
// ����˵��     uartn           ����ģ���(UART_0,UART_1,UART_2,UART_3)
// ����˵��     baud            ���ڲ�����
// ����˵��     tx_pin          ���ڷ�������
// ����˵��     rx_pin          ���ڽ�������
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void Bluetooth_uart_init(uart_index_enum uartn, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin)
{
    // ���ڳ�ʼ��
    uart_init(uartn, baud, tx_pin, rx_pin);

    // ���������ж�
    uart_tx_interrupt(uartn, 1);
    uart_rx_interrupt(uartn, 1);

    // Bluetooth_data���ݳ�ʼ��
    Bluetooth_data.receiveFinished = false;
    Bluetooth_data.receiveStart = false;
    Bluetooth_data.receiveIndex = 0;
    Bluetooth_data.data_verity = 0;
    Bluetooth_data.data_choice = false;         //Ĭ�ϱջ�ģʽ
    Bluetooth_data.copyready = false;           //��ʼʱ����δ���յ�
    Bluetooth_data.Flag_Bluetooth = false;      //���ߴ��ڶ�ʱ���ͱ�־λ
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ͨ���������ֻ��������ݣ�֧�� int��float ����
// ����˵��     string  ��Ҫ���͵���������д��
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void Bluetooth_Send(char *string, ...)
{
    uart_write_byte_wait(UART_0,0XA5);          //�������ݰ�ͷ
    char *ptr_string = string;                  //����ֱ��ʹ��string������stringָ��ĵ�ַ�����仯
    int int_data;
    float float_data;
    void* void_ptr = &float_data;
    uint8_t* float_buff = (uint8_t*)void_ptr;
    va_list ap;                                 //����һ��va_list���ͱ���ap
    va_start(ap, string);                       //��ʼ��va��ʹvaָ��string����һ����ַ
    uint8_t sum = 0x00;                         //�����У������λ
    uint8_t sum_buff = 0;                       //����32λת��Ϊ8λ
    while(*ptr_string != '\0')
    {
        if(*ptr_string == '%')                  //��ʽ���Ʒ�
        {
            switch(*++ptr_string)
            {
                case 'd':
                    int_data = va_arg(ap, int);  //��������
                    for (int i = 0; i < 4; i++)
                    {
                        sum_buff = (uint32_t)int_data & (0xFF << 8*i);
                        sum += sum_buff;
                        uart_write_byte_wait(UART_0,sum_buff);
                    }
                    break;
                case 'f':
                    float_data = va_arg(ap, float);
                    for (int i = 0; i < 4; i++)
                    {
                        sum_buff = float_buff[i];
                        sum += sum_buff;
                        uart_write_byte_wait(UART_0,sum_buff);
                    }
                    break;
                default:
                    break;
            }
        }
        ptr_string++;  //ָ����һ����ַ
    }
    uart_write_byte_wait(UART_0,sum);       //���ͺ�У��
    uart_write_byte_wait(UART_0,0x5A);      //�������ݰ�β
    va_end(ap);                             //�ر�ap
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ͨ�����ߴ��ڷ������ݣ���������λ����ָ�����ݽ��м��ӷ���
// ����˵��     data Ҫ���͵�����
// ���ز���     void
// ˵��   ���������
        /*Wireless_Uart_Send((int32_t)(motorStr.Set_current_motor*100000),
        (int32_t)(icarStr.SpeedSet*100000),
        (int32_t)(icarStr.speed_set*100000),
        (int32_t)(icarStr.SpeedFeedback*100000));*/
//-------------------------------------------------------------------------------------------------------------------
void Wireless_Uart_Send(int32_t data1,int32_t data2,int32_t data3,int32_t data4)
{
    int cnt=0;
    unsigned char i;
    unsigned char sum = 0;
    unsigned char add = 0;
    uint8_t buff[28] = {0};

    buff[cnt++]  = 0xAA;
    buff[cnt++]  = 0xFF;
    buff[cnt++]  = 0xF1;
    buff[cnt++]  = 0x16;

    Byte4_Union byte_4_union;

    byte_4_union.U32 = (uint32_t)data1;
    for(int i=0;i<4;i++)
    {
        buff[cnt++]=byte_4_union.U8_Buff[i];
    }

    byte_4_union.U32 = (uint32_t)data2;
    for(int i=0;i<4;i++)
    {
        buff[cnt++]=byte_4_union.U8_Buff[i];
    }

    byte_4_union.U32 = (uint32_t)data3;
    for(int i=0;i<4;i++)
    {
        buff[cnt++]=byte_4_union.U8_Buff[i];
    }

    byte_4_union.U32 = (uint32_t)data4;
    for(int i=0;i<4;i++)
    {
        buff[cnt++]=byte_4_union.U8_Buff[i];
    }

    for( i=0;i<6;i++)
    {
        buff[cnt++]  = 0x00;
    }

    for( i=0;i<cnt;i++)
    {
        sum += buff[i];
        add += sum;
    }

    buff[cnt++]=sum;
    buff[cnt++]=add;

    uart_write_buffer(UART_0,buff,cnt);
}


//-------------------------------------------------------------------------------------------------------------------
// �������       ��edgeboardͨ�ŵĺ����ӿڣ����ݰ����ܺ���
// ����˵��       void
// ���ز���       void
// ˵��          �����޶���
//-------------------------------------------------------------------------------------------------------------------
void uart2_rx_interrupt_handler(void)
{
    //����һ���������������ݣ�������
    uint8 temp = 0;
    //�������ݲ�ѯʽ,�����ݻ᷵��TRUE,û�����ݻ᷵��FALSE.
    if(uart_query_byte(eb_using_uart, &temp))
    {
        //��⵽֡ͷ
        if(temp == USB_FRAME_HEAD && !usbStr.receiveStart)
        {
            //��ʼ����һ֡����
            usbStr.receiveStart = true;
            //д�����ݣ���ָ����һ��
            usbStr.receiveBuff[0] = temp;
            usbStr.receiveIndex = 1;
            //Ĭ��֡����
            usbStr.receiveBuff[2] = USB_FRAME_LENMIN;
        }
        else if(usbStr.receiveIndex == 2)
        {
            //����֡��������
            usbStr.receiveBuff[usbStr.receiveIndex] = temp;
            usbStr.receiveIndex++;
            //֡��������
            if(temp > USB_FRAME_LENMAX || temp < USB_FRAME_LENMIN)
            {
                //�������ֹͣ�������ݣ�Ĭ��֡��Ϊ��С
                usbStr.receiveBuff[2] = USB_FRAME_LENMIN;
                usbStr.receiveIndex = 0;
                usbStr.receiveStart = false;
            }
        }
        else if(usbStr.receiveStart && usbStr.receiveIndex < USB_FRAME_LENMAX)
        {
            //���վ����������Ϣ
            usbStr.receiveBuff[usbStr.receiveIndex] = temp;
            usbStr.receiveIndex++;
        }

        //һ֡���ݽ������
        if((usbStr.receiveIndex >= USB_FRAME_LENMAX || usbStr.receiveIndex >= usbStr.receiveBuff[2]) && usbStr.receiveIndex > USB_FRAME_LENMIN)
        {
            //�����У��λ
            usbStr.data_verity = 0;
            //����֡��������Ĭ��Ϊ��С
            uint8 length = USB_FRAME_LENMIN;
            //��ȡ֡��
            length = usbStr.receiveBuff[2];
            //���к�У�����
            for(int i=0; i<length-1; i++)
                usbStr.data_verity += usbStr.receiveBuff[i];
            //���к�У��
            if(usbStr.data_verity == usbStr.receiveBuff[length-1])
            {
                //У��ɹ���������ȷ���������ݵ�receiveBuffFinished�����ı��������״̬
                memcpy(usbStr.receiveBuffFinished,usbStr.receiveBuff,USB_FRAME_LENMAX);
                usbStr.receiveFinished = true;

                //���ܳ�����ָ�����⴦������ʵʱ�ԣ���USB_ADDR_CONTROL->�ٶȺͷ���ָ��
                if(usbStr.receiveBuffFinished[1]  == USB_ADDR_CONTROL)
                {
                    //�������������������ݴ�������ת����
                    Byte2_Union byte_2_union;
                    Byte4_Union byte_4_union;
                    //���ٶ����ݸ�ֵ��������
                    for(int i=0; i<4; i++)
                        byte_4_union.U8_Buff[i] = usbStr.receiveBuffFinished[3+i];
                    //���������ݸ�ֵ��������
                    byte_2_union.U8_Buff[0] = usbStr.receiveBuffFinished[7];
                    byte_2_union.U8_Buff[1] = usbStr.receiveBuffFinished[8];
                    //����ת���ʵʱ�ԣ�һ�õ�ת����Ϣ�Ϳ�ʼת��
                    SERVO_SetPwmValueCorrect(byte_2_union.U16);
                    //���ٶȺͷ�����Ϣ���д洢
                    icarStr.SpeedSet = byte_4_union.Float;         //�ٶ�
                    icarStr.ServoPwmSet = byte_2_union.U16;        //����
                }

                //��λ������ͨ�ųɹ�
                if(!usbStr.connected)
                {
                    //������ʾ�����ӳɹ�
                    Buzzer_Enable(BuzzerOk);
                    usbStr.connected = true;
                }
                //δ���ߣ����߼�������
                usbStr.counterDrop = 0;
            }
            else
            {
                //���У����󣬷�ת��ƽ
                gpio_toggle_level(P21_5);
            }

            //������տ�����Ϣ���ȴ���һ����Ϣ�Ľ���
            usbStr.receiveIndex = 0;
            usbStr.receiveStart = false;
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������       ��ʼ�׶ε���ʱ���������ԣ���Ӧ��ͨ�Žӿ�
// ����˵��       void
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
void uart0_rx_interrupt_handler(void)
{
    //����һ���������������ݣ�������
    uint8 temp = 0;
    //�������ݲ�ѯʽ,�����ݻ᷵��TRUE,û�����ݻ᷵��FALSE
    if(uart_query_byte(bluetooth_using_uart, &temp))
    {
        //������յ�����Ϊ֡ͷ�����һ�û���ǽ��գ����н������Ľ���
        if(temp == DATA_PEAK_HEAD && !Bluetooth_data.receiveStart)
        {
            //�л�Ϊ��ʼ����״̬
            Bluetooth_data.receiveStart = true;
            //д������
            Bluetooth_data.receiveBuff[0] = temp;
            //������һ����������
            Bluetooth_data.receiveIndex = 1;
            //����У��λ
            Bluetooth_data.data_verity = 0;
            //��ʼ��������
            Bluetooth_data.receiveFinished = false;
        }
        else if(Bluetooth_data.receiveStart && Bluetooth_data.receiveIndex < UART_FRAME_LEN - 2)
        {
            //д����յ�������
            Bluetooth_data.receiveBuff[Bluetooth_data.receiveIndex] = temp;
            //���кż�1
            Bluetooth_data.receiveIndex++;
            //���к�У�����ݼӺ�
            Bluetooth_data.data_verity += temp;
        }
        else if(Bluetooth_data.receiveIndex == UART_FRAME_LEN - 2)
        {
            //д����յ�������
            Bluetooth_data.receiveBuff[Bluetooth_data.receiveIndex] = temp;
            //���кż�һ
            Bluetooth_data.receiveIndex++;

            //���к�У��
            if(Bluetooth_data.data_verity != temp)
            {
                //���ݳ��ִ���
                Bluetooth_data.receiveIndex = 0;
                Bluetooth_data.receiveStart = FALSE;
                //����ֵ-1
                uart_write_string(bluetooth_using_uart, "-1");
            }
        }
        else if(Bluetooth_data.receiveIndex == UART_FRAME_LEN - 1 && Bluetooth_data.receiveStart && temp == DATA_PEAK_TAIL)
        {
            //д����յ�������
            Bluetooth_data.receiveBuff[Bluetooth_data.receiveIndex] = temp;
            //���кż�һ
            Bluetooth_data.receiveIndex++;
            //������ݽ��գ�����У��ͨ��
            Bluetooth_data.receiveFinished = TRUE;
        }

        //����������֡���������ݴ���
        if(Bluetooth_data.receiveIndex >= UART_FRAME_LEN)
        {
            //У��ɹ���������ȷ���������ݵ�receiveBuffFinished�����ı��������״̬
            memcpy(Bluetooth_data.receiveBuffFinished,Bluetooth_data.receiveBuff,UART_FRAME_LEN);
            Bluetooth_data.copyready = true;

            //����״̬���
            Bluetooth_data.receiveStart = FALSE;
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������       ����2�����жϺ��������ӿ�
// ����˵��       void
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    // �����ж�Ƕ��
    interrupt_global_enable(0);
    // �򿪽��մ���2
    IfxAsclin_Asc_isrReceive(&uart2_handle);
    // ���ڽ��մ���
    uart2_rx_interrupt_handler();
}


//-------------------------------------------------------------------------------------------------------------------
// �������       ����0�����жϺ��������ӿ�
// ����˵��       void
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    // �����ж�Ƕ��
    interrupt_global_enable(0);
    // �򿪽��մ���0
    IfxAsclin_Asc_isrReceive(&uart0_handle);
    // ���ڽ��մ���
    uart0_rx_interrupt_handler();
}


//-------------------------------------------------------------------------------------------------------------------
// �������       Edgeboardͨ�ż������߳̿�����
// ����˵��       void
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
void USB_Edgeboard_Timr(void)
{
    //Edgeboardͨ�ŵ��߼��
    if(usbStr.connected)
    {
        //���Ϊ����״̬����ƣ�˵��Ϊ����
        gpio_low(P20_9);
        //��ǰ���ӳɹ��ˣ�Ϊ����״̬�����һֱ��ͨ�ţ������ж��в���������߼�����
        usbStr.counterDrop++;
        //������߼�����ʱ������3s��˵��û����ͨ�ţ��ж�Ϊ���ߣ��ı���ص�״̬
        if(usbStr.counterDrop > 3000)
        {
            //������״̬��Ϊ����
            usbStr.connected = false;
            //����Լ����
            usbStr.inspectorEnable = false;
        }
        //���ʹ����������Լ죬�������в���
        if(usbStr.inspectorEnable)
        {
            usbStr.counterSend++;
        }
    }
    else
    {
        //��ƣ�˵�����ӶϿ�
        gpio_high(P20_9);
        //����֮�����pid�Ļ������
        PID_clear(&car_speed_pid);
    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������       ���ߴ����̼߳�����
// ����˵��       void
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
void Wireless_Timer(void)
{
    // ���徲̬����������
    static uint8_t num;
    num++;

    // ÿ5ms���ߴ��ڻ���������һ������
    if(num >= 50)
    {
        Wireless_data.Flag_Wireless = true;
//        Bluetooth_data.Flag_Bluetooth = true;
        num = 0;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������       Edgeboardͨ�Ŵ�����
// ����˵��       void
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
void USB_Edgeboard_Handle(void)
{
    //���ݽ��ճɹ�
    if(usbStr.receiveFinished)
    {
        //��ʼ���ݴ����ı����״̬��������һ�����ݽ���
        usbStr.receiveFinished = false;
        //�������������������ݴ���
        Byte2_Union byte2_union;
        Byte4_Union byte4_union;
        Byte4_Union byte4_union_errsp;//���ڽ��ջ��ַ���������ֵ

        //�����ݣ������ݵĵ�ַ����λΪ1-> ��ַ & 0x80 -> xxxx xxxx & 1000 0000 ->�����ַ�����λΪ1����֮��Ϊ��0����Ϊ�棻�����ַ���λΪ0����֮��ֵΪ0����Ϊ�١�
        if(usbStr.receiveBuffFinished[1] & 0x80)
        {
            //��ȡ����֡�ĵ�ַ��Ϣ��������ַ���λ��1
            uint8 Addr = (uint8)(usbStr.receiveBuffFinished[1] & 0x7F);
            //��Ӧ��ַ��Ϣ�Ĵ��� ��֪��Ϊʲô��Ӧ������û�� ����Ϊ�����Ч��ֱ��ע�͵� ������ͨͨ������
//            switch(Addr)
//            {
//                //�����Ϣ
//                case USB_ADDR_BATTERY:
//                    break;
//                //�����ֵ
//                case USB_ADDR_SERVOTHRESHOLD:
//                    break;
//            }
        }
        else
        {
            //���ݵ�ַ��Ϣ��д���ݵ���λ��
            switch(usbStr.receiveBuffFinished[1])
            {
                //�����ֵ 0X03
                case USB_ADDR_SERVOTHRESHOLD:
                    //����uint16���͵����ݴ���
                    if(usbStr.receiveBuffFinished[3] == 1)
                    {
                        //��ת��ֵ
                        byte2_union.U8_Buff[0] = usbStr.receiveBuffFinished[4];
                        byte2_union.U8_Buff[1] = usbStr.receiveBuffFinished[5];
                        servoStr.thresholdLeft = byte2_union.U16;
                        //�´�flash�洢����
                        flashSaveEnable = true;
                        //��������ʾ
                        Buzzer_Enable(BuzzerDing);
                    }
                    else if(usbStr.receiveBuffFinished[3] == 2)
                    {
                        //��ת��ֵ
                        byte2_union.U8_Buff[0] = usbStr.receiveBuffFinished[4];
                        byte2_union.U8_Buff[1] = usbStr.receiveBuffFinished[5];
                        servoStr.thresholdRight = byte2_union.U16;
                        //�´�flash�洢����
                        flashSaveEnable = true;
                        //��������ʾ
                        Buzzer_Enable(BuzzerDing);
                    }
                    else if(usbStr.receiveBuffFinished[3] == 3)     //��ֵ
                    {
                        //��ֵ
                        byte2_union.U8_Buff[0] = usbStr.receiveBuffFinished[4];
                        byte2_union.U8_Buff[1] = usbStr.receiveBuffFinished[5];
                        servoStr.thresholdMiddle = byte2_union.U16;
                        //�´�flash�洢����
                        flashSaveEnable = true;
                        //��������ʾ
                        Buzzer_Enable(BuzzerDing);
                    }
                    break;
                //PID�������� 0X10
                case USB_ADDR_PID_INFORMATION:
                    // speed-loop-kp����ֵ
                    for(int i=0;i<4;i++)
                        byte4_union.U8_Buff[i] = usbStr.receiveBuffFinished[3+i];
                    icarStr.speed_loop_Kp = byte4_union.Float;
                    // speed-loop-ki����ֵ
                    for(int i=0;i<4;i++)
                        byte4_union.U8_Buff[i] = usbStr.receiveBuffFinished[7+i];
                    icarStr.speed_loop_Ki = byte4_union.Float;
                    // speed-loop-kd����ֵ
                    for(int i=0;i<4;i++)
                        byte4_union.U8_Buff[i] = usbStr.receiveBuffFinished[11+i];
                    icarStr.speed_loop_Kd = byte4_union.Float;
                    // current-loop-kp����ֵ
                    for(int i=0;i<4;i++)
                        byte4_union.U8_Buff[i] = usbStr.receiveBuffFinished[15+i];
                    icarStr.current_loop_Kp = byte4_union.Float;
                    // current-loop-kp����ֵ
                    for(int i=0;i<4;i++)
                        byte4_union.U8_Buff[i] = usbStr.receiveBuffFinished[19+i];
                    icarStr.current_loop_Ki = byte4_union.Float;
                    // current-loop-kp����ֵ
                    for(int i=0;i<4;i++)
                        byte4_union.U8_Buff[i] = usbStr.receiveBuffFinished[23+i];
                    icarStr.current_loop_Kd = byte4_union.Float;

                    // ����KPֵ�ж��Ƿ񿪻����Ǳջ�,0��ʾ��������0��ʾ�ջ�
                    if(icarStr.speed_loop_Kp != 0)
                        motorStr.CloseLoop = true;
                    else
                        motorStr.CloseLoop = false;

                    // д��flash
                    flashPIDEnable = true;
                    // ��������ʾ
                    Buzzer_Enable(BuzzerOk);
                    break;
                //��������Ч
                case USB_ADDR_BUZZER:
                    if(usbStr.receiveBuffFinished[3] == 1)          //OK
                        Buzzer_Enable(BuzzerOk);
                    else if(usbStr.receiveBuffFinished[3] == 2)     //Warnning
                        Buzzer_Enable(BuzzerWarnning);
                    else if(usbStr.receiveBuffFinished[3] == 3)     //Finish
                        Buzzer_Enable(BuzzerFinish);
                    else if(usbStr.receiveBuffFinished[3] == 4)     //Ding
                        Buzzer_Enable(BuzzerDing);
                    else if(usbStr.receiveBuffFinished[3] == 5)     //SystemStart
                        Buzzer_Enable(BuzzerSysStart);
                    break;
                //LED��Ч
                case USB_ADDR_LIGHT:
                    //�����ݸ�ֵ�������壬�������ݴ���
                    for(int i=0;i<4;i++)
                        byte4_union.U8_Buff[i] = usbStr.receiveBuffFinished[3+i];
                    break;
                //�ٿ�ģʽ�л�
                case USB_ADDR_SPEEDMODE:
                    //5.4 �ջ���������
                    if(usbStr.receiveBuffFinished[3] == 1)
                    {
                        motorStr.CloseLoop = true;
                        car_speed_pid.mode_t=PID_DELTA;
                        icarStr.SpeedSet = 0;

                    }
                    else if(usbStr.receiveBuffFinished[3] == 2)
                    {
                        //5.4 �ջ�λ��ģʽ
                        car_speed_pid.mode_t=PID_POSITION;
                        motorStr.CloseLoop = true;
                        icarStr.SpeedSet = 0;
                    }
                    else if(usbStr.receiveBuffFinished[3] == 3)
                    {
                        //���ַ�����ֵ��ֵ
                        for(int i=0;i<4;i++)
                              byte4_union_errsp.U8_Buff[i] = usbStr.receiveBuffFinished[4+i];
                        car_speed_pid.separationThreshold =  byte4_union_errsp.Float;
                    }
                    //�´�flash�洢����
                    flashSaveEnable = true;
//                    icarStr.SpeedSet = 0;
                    //��������ʾ
                    Buzzer_Enable(BuzzerDing);
                    break;
//-----------------------------[�Լ�������]-------------------------------------------
                case USB_ADDR_INSPECTOR:                                   //�Լ��������
                    usbStr.inspectorEnable = true;
                    break;
                case USB_ADDR_SELFCHECK:                                   //��ʼ�Լ�
                    //ICAR_SelfcheckControl(usbStr.receiveBuffFinished[3]);
                    break;
            }
        }
    }
//-----------------------[�Լ�������ݷ���]-----------------------------
    if(usbStr.inspectorEnable && usbStr.connected && usbStr.counterSend > 150)//150ms
    {
        USB_Edgeboard_ServoThreshold(1);          //���Ͷ����ֵ
        system_delay_ms(1);
        USB_Edgeboard_ServoThreshold(2);
        system_delay_ms(1);
        USB_Edgeboard_ServoThreshold(3);
        system_delay_ms(1);
        USB_Edgeboard_CarSpeed();                 //���ͳ���
        usbStr.counterSend = 0;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������       Bluetoothͨ�Ŵ�����
// ����˵��       void
// ���ز���       void
// ˵����Ϊ���㿴����ȫ����Ϊע��
//-------------------------------------------------------------------------------------------------------------------
void use_bluetooth_Handle(void)
{
//    //�����ж��Ƿ�Ϊ��һ�ν��봦�������Ϣ
//    static bool is_first = true;
//    //������������������Ϣ
//    Byte4_Union byte4_union;
//
//    //������ݽ��ճɹ�
//    if(Bluetooth_data.copyready)
//    {
//        //ȡ������->���ջ�ѡ��
//        Bluetooth_data.data_choice = Bluetooth_data.receiveBuffFinished[1];
//
//        //������������ת������
//        Byte4_Union byte_4_union;
//        //��ȡ����speed
//        for(int i=0; i<4; i++)
//            byte_4_union.U8_Buff[i] = Bluetooth_data.receiveBuffFinished[14+i];
//        Bluetooth_data.data_speed = byte_4_union.Float;
//        //��ȡ����angle
//        for(int i=0; i<4; i++)
//            byte_4_union.U8_Buff[i] = Bluetooth_data.receiveBuffFinished[18+i];
//        Bluetooth_data.data_angle = byte_4_union.Float;
//        //���ʵʱ����
//        servo_contral(Bluetooth_data.data_angle);
//
//        if(is_first)
//        {
//            //�ı�״̬�����ǵ�һ�������������ˣ������ظ�����pid����
//            is_first = false;
//
//            //��ȡ����ٶȻ�kp����
//            for(int i=0; i<4; i++)
//                byte4_union.U8_Buff[i] = Bluetooth_data.receiveBuffFinished[2+i];
//            Bluetooth_data.motor_speed_loop_Kp = byte4_union.Float;
//            //��ȡ����ٶȻ�ki����
//            for(int i=0; i<4; i++)
//                byte4_union.U8_Buff[i] = Bluetooth_data.receiveBuffFinished[6+i];
//            Bluetooth_data.motor_speed_loop_Ki = byte4_union.Float;
//            //��ȡ����ٶȻ�kd����
//            for(int i=0; i<4; i++)
//                byte4_union.U8_Buff[i] = Bluetooth_data.receiveBuffFinished[10+i];
//            Bluetooth_data.motor_speed_loop_Kd = byte4_union.Float;
//            //��ȡ���������kp����
//            for(int i=0; i<4; i++)
//                byte4_union.U8_Buff[i] = Bluetooth_data.receiveBuffFinished[22+i];
//            Bluetooth_data.motor_current_loop_Kp = byte4_union.Float;
//            //��ȡ���������ki����
//            for(int i=0; i<4; i++)
//                byte4_union.U8_Buff[i] = Bluetooth_data.receiveBuffFinished[26+i];
//            Bluetooth_data.motor_current_loop_Ki = byte4_union.Float;
//            //��ȡ���������kd����
//            for(int i=0; i<4; i++)
//                byte4_union.U8_Buff[i] = Bluetooth_data.receiveBuffFinished[30+i];
//            Bluetooth_data.motor_current_loop_Kd = byte4_union.Float;
//
//            //�ȴ����ݽ�����ɣ���ƣ�˵�����ݽ������
//            gpio_init(P20_9, GPO, 0, GPO_PUSH_PULL);
//            //��ȡpid����
//            float pid_data[6] = {Bluetooth_data.motor_speed_loop_Kp, Bluetooth_data.motor_speed_loop_Ki,
//                                 Bluetooth_data.motor_speed_loop_Kd, Bluetooth_data.motor_current_loop_Kp,
//                                 Bluetooth_data.motor_current_loop_Ki, Bluetooth_data.motor_current_loop_Kd};
//            // д��pid������flash��
//            my_flash_write_pid(pid_data);
//            // ��ʼ��speed_loop_pid����
//            float speed_loop_pid_buff_bluetooth[3] = {Bluetooth_data.motor_speed_loop_Kp, Bluetooth_data.motor_speed_loop_Ki, Bluetooth_data.motor_speed_loop_Kd};
//            PID_Init(&car_speed_pid, PID_POSITION, speed_loop_pid_buff_bluetooth, SPEED_LOOP_MAX_OUT, SPEED_LOOP_P_MAX_OUT);
//            // ��ʼ��current_loop_pid����
//            float current_loop_pid_buff_bluetooth[3] = {Bluetooth_data.motor_current_loop_Kp, Bluetooth_data.motor_current_loop_Ki, Bluetooth_data.motor_current_loop_Kd};
//            PID_Init(&car_current_pid, PID_POSITION, current_loop_pid_buff_bluetooth, CURRENT_LOOP_MAX_OUT, CURRENT_LOOP_P_MAX_OUT);
//
//            // ��������ʾ
//            Buzzer_Enable(BuzzerSysStart);
//        }
//    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������       ���ߴ��ڻ�������ʱ����
// ����˵��       void
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
void Wireless_Handle(void)
{
    // ���ߴ��ڶ�ʱ��������
    static num_time = 0;//���ڼ�¼��������
    if(Wireless_data.Flag_Wireless == true)
    {
//        Wireless_Uart_Send((int32_t)(motorStr.Set_current_motor*100000),
//                           (int32_t)(icarStr.SpeedSet*100000),
//                           (int32_t)(icarStr.speed_set*100000),
//                           (int32_t)(icarStr.SpeedFeedback*100000));
        printf("SpeedSet:%f,speed_set:%f,SpeedFeedback:%f,error%f,\n",
             /*(int32_t)(motorStr.Set_current_motor*100000),*/
//               (int32_t)(icarStr.SpeedSet*1000),
//               (int32_t)(icarStr.speed_set*1000),
//               (int32_t)(icarStr.SpeedFeedback*1000)
                icarStr.SpeedSet,icarStr.speed_set,icarStr.SpeedFeedback,car_speed_pid.error[0]);

//        printf("I:%f",car_speed_pid.Iout);

        Wireless_data.Flag_Wireless = false;
    }

//    // ������ʱ��������
//    if(Bluetooth_data.Flag_Bluetooth == true)
//    {
//        Bluetooth_Send("%f%f%f", icarStr.SpeedSet, icarStr.SpeedFeedback, car_speed_pid.out);
//        Bluetooth_data.Flag_Bluetooth = false;
//    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������       Edgeboard���Ͱ����ź�
// ����˵��       time: ����ʱ��
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
void USB_Edgeboard_TransmitKey(uint16 time)
{
    //�����͵�����
    uint8 check = 0;
    uint8 buff[6];
    Byte2_Union byte2_union;

    //֡ͷ
    buff[0] = 0x42;
    //��ַ
    buff[1] = USB_ADDR_KEYINPUT;
    //֡��
    buff[2] = 0x06;

    //ת������ʱ������д������
    byte2_union.U16 = time;
    buff[3] = byte2_union.U8_Buff[0];
    buff[4] = byte2_union.U8_Buff[1];

    //���к�У��
    for(int i=0;i<5;i++)
        check += buff[i];
    //д���У������
    buff[5] = check;
    //��������
    uart_write_buffer(eb_using_uart, buff, 6);
}


//================================[UNIT-���������Լ����ͨ������]==================================
//-------------------------------------------------------------------------------------------------------------------
// �������       ���Ͷ����ֵ
// ����˵��       chanel: 1/��ת��ֵ��2/��ת��ֵ��3/��ֵ
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
void USB_Edgeboard_ServoThreshold(uint8 chanel)
{
    //����ͨ��ѡ���˳��ú���
    if(chanel<1 || chanel>3)
        return;

    //�����͵�����
    Byte2_Union byte2_union;
    uint8 check = 0;
    uint8 buff[9];
    //֡ͷ
    buff[0] = 0x42;
    //��ַ
    buff[1] = USB_ADDR_SERVOTHRESHOLD;
    //֡��
    buff[2] = 0x07;
    //ͨ��
    buff[3] = chanel;

    //ѡ��ͨ������д�뷢����Ϣ
    switch(chanel)
    {
        case 1:
        {
            byte2_union.U16 = servoStr.thresholdLeft;
            buff[4] = byte2_union.U8_Buff[0];
            buff[5] = byte2_union.U8_Buff[1];
            break;
        }
        case 2:
        {
            byte2_union.U16 = servoStr.thresholdRight;
            buff[4] = byte2_union.U8_Buff[0];
            buff[5] = byte2_union.U8_Buff[1];
            break;
        }
        case 3:
        {
            byte2_union.U16 = servoStr.thresholdMiddle;
            buff[4] = byte2_union.U8_Buff[0];
            buff[5] = byte2_union.U8_Buff[1];
            break;
        }
    }

    //��У����Ϣ
    for(int i=0;i<6;i++)
        check += buff[i];
    //д���У������
    buff[6] = check;
    //��������
    uart_write_buffer(eb_using_uart, buff, 9);
}


//-------------------------------------------------------------------------------------------------------------------
// �������       ����С���ٶ�
// ����˵��       void
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
void USB_Edgeboard_CarSpeed(void)
{
    //�����͵�����
    Byte4_Union byte4_union;
    uint8_t check = 0;
    uint8_t buff[8];

    //֡ͷ
    buff[0] = 0x42;
    //��ַ
    buff[1] = USB_ADDR_SPEEDBACK;
    //֡��
    buff[2] = 0x08;

    //���͵��ٶ���Ϣת��
    byte4_union.Float = icarStr.SpeedFeedback;
    buff[3] = byte4_union.U8_Buff[0];
    buff[4] = byte4_union.U8_Buff[1];
    buff[5] = byte4_union.U8_Buff[2];
    buff[6] = byte4_union.U8_Buff[3];

    //У�����Ϣ
    for(int i=0;i<7;i++)
        check += buff[i];
    //д���У������
    buff[7] = check;
    //��������
    uart_write_buffer(eb_using_uart, buff, 8);
}


//==============================[����λ������ͨ������]====================================
//-------------------------------------------------------------------------------------------------------------------
// �������     ����λ����������
// ����˵��     senddata  ��Ҫ���͵���������д��
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void senddata_to_upper(uint8_t addr,float senddata)
{
    //�����͵�����
    Byte4_Union byte4_union;
    uint8_t check = 0;
    uint8_t buff[8];

    //֡ͷ
    buff[0] = 0x42;
    //��ַ
    buff[1] = addr;
    //֡��
    buff[2] = 0x08;

    //���͵��ٶ���Ϣת��
    byte4_union.Float = senddata;
    buff[3] = byte4_union.U8_Buff[0];
    buff[4] = byte4_union.U8_Buff[1];
    buff[5] = byte4_union.U8_Buff[2];
    buff[6] = byte4_union.U8_Buff[3];

    //У�����Ϣ
    for(int i=0;i<7;i++)
        check += buff[i];
    //д���У������
    buff[7] = check;
    //��������
    uart_write_buffer(eb_using_uart, buff, 8);
}




