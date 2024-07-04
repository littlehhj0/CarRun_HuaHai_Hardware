/*
 * uart.h
 *
 *  Created on: 2023��3��18��
 *      Author: wzl
 */


#ifndef CODE_UART_UART_H_
#define CODE_UART_UART_H_


//����ͷ�ļ�
#include <stdarg.h>
#include "cpu0_main.h"


//�궨���������ڡ����ߴ������ţ�û��ͬʱ�����ߴ��ں�������
#define uart_booluteeth_pin_tx      0       //UART0_TX_P14_0
#define uart_booluteeth_pin_rx      0       //UART0_RX_P14_1
#define uart_wireless_pin_tx        0       //UART0_TX_P14_0
#define uart_wireless_pin_rx        0       //UART0_RX_P14_1
//�궨�����������ߴ���ʹ�õĴ��ڣ�û��ͬʱ�����ߴ��ں��������������������õ�ͬһ�鴮�ڣ�ʹ�ñ�Ĵ���ֻ��Ҫ�ĺ궨�壩
#define bluetooth_using_uart        0       //UART_0
#define wireless_using_uart         0       //UART_0
//�궨�����������ߴ���ʹ�ô��ڵĲ�����
#define bluetooth_using_uart_baud   115200
#define wireless_using_uart_baud    115200


//�궨������λ������ͨ������
#define uart_eb_pin_tx              14      //UART2_TX_P10_5
#define uart_eb_pin_rx              10      //UART2_RX_P10_6
//�궨��edgeboardʹ�õĴ���
#define eb_using_uart               2       //UART_2
//�궨��edgeboardʹ�ô��ڵĲ�����
#define eb_using_uart_baud          460800


//����֡ͷ--����
#define DATA_PEAK_HEAD              0xa5
//����֡β--����
#define DATA_PEAK_TAIL              0x5a
//ͨ�������ֽڳ���--����
#define UART_FRAME_LEN              36


//���ߴ��ڵ������ݽṹ��
typedef struct
{
    bool  Flag_Wireless;                                    //���ݷ��Ͷ�ʱ��־λ
}Wireless_data_recevie_Struct;


//����--�����������ݽṹ��
typedef struct
{
    bool  receiveStart;                                     //���ݽ��տ�ʼ
    uint8 receiveIndex;                                     //��������
    bool  receiveFinished;                                  //���ݶ��н��ղ�У�����
    bool  copyready;                                        //���ݿ������
    bool  Flag_Bluetooth;                                   //���ݷ��Ͷ�ʱ

    uint8 data_verity;                                      //����У��λ
    uint8 receiveBuff[UART_FRAME_LEN];                      //���ݽ��ն���
    uint8 receiveBuffFinished[UART_FRAME_LEN];              //���ݽ��ն��У�У��ɹ�

    bool  data_choice;                                      //�������ջ���׼λ
    float motor_speed_loop_Kp;                              //����ٶȻ�kp
    float motor_speed_loop_Ki;                              //����ٶȻ�ki
    float motor_speed_loop_Kd;                              //����ٶȻ�kd
    float motor_current_loop_Kp;                            //���������kp
    float motor_current_loop_Ki;                            //���������ki
    float motor_current_loop_Kd;                            //���������kd
    float data_speed;                                       //����->�ٶ�
    float data_angle;                                       //����->�Ƕ�
}BlueTooth_data_recevie_Struct;


//ͨ�������ֽ������
#define USB_FRAME_LENMAX            32
//USBͨ�������ֽ���̳���
#define USB_FRAME_LENMIN            4
//ebͨ������֡ͷ
#define USB_FRAME_HEAD              0x42
//ͨ�ŵ�ַ
#define USB_ADDR_HEART              0x00                //����������
#define USB_ADDR_CONTROL            0x01                //���ܳ�����
#define USB_ADDR_SPEEDMODE          0x02                //�ٿ�ģʽ
#define USB_ADDR_SERVOTHRESHOLD     0x03                //�����ֵ
#define USB_ADDR_BUZZER             0x04                //��������Ч
#define USB_ADDR_LIGHT              0x05                //LED��Ч
#define USB_ADDR_KEYINPUT           0x06                //��������
#define USB_ADDR_BATTERY            0x07                //�����Ϣ
#define USB_ADDR_PID_INFORMATION    0x10                //��λ��PID������Ϣ
//�����Լ����ͨ��ʹ��
#define USB_ADDR_INSPECTOR          0x0A                //���ܳ��Լ������������
#define USB_ADDR_SELFCHECK          0x0B                //���ܳ��Լ쿪ʼ
#define USB_ADDR_SPEEDBACK          0x08                //������Ϣ������m/s


//ebͨ�Žṹ��
typedef struct
{
    bool   receiveStart;                                     //���ݽ��տ�ʼ
    uint8  receiveIndex;                                     //��������
    bool   receiveFinished;                                  //���ݶ��н��ղ�У�����
    uint8  receiveBuff[USB_FRAME_LENMAX];                    //USB���ն��У���ʱ����
    uint8  receiveBuffFinished[USB_FRAME_LENMAX];            //USB���ն��У�У��ɹ�
    uint16 counter;                                          //������
    uint16 counterDrop;                                      //���߼�����
    uint16 counterSend_upper;                                //����λ�������ٶȼ�ʱ��

    uint8  data_verity;                                      //����У��

    uint16 counterSend;                                      //�Լ����ݷ��ͼ�����
    bool   connected;                                        //��λ��ͨ������״̬
    bool   inspectorEnable;                                  //���������Լ��������ʹ��
}UsbStruct;


//����ebͨ�����ݽṹ��
extern UsbStruct usbStr;
//�����������ݽṹ��
extern BlueTooth_data_recevie_Struct Bluetooth_data;
//�������ߴ������ݽṹ��
extern Wireless_data_recevie_Struct Wireless_data;


//==================================================UART ��������==================================================
void USB_uart_init                  (uart_index_enum uartn, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin);
void Wireless_uart_init             (uart_index_enum uartn, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin);
void Bluetooth_uart_init            (uart_index_enum uartn, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin);

void use_bluetooth_Handle           (void);
void Bluetooth_Send                 (char *string, ...);

void Wireless_Uart_Send             (int32_t data1,int32_t data2,int32_t data3,int32_t data4);
void Wireless_Handle                (void);
void Wireless_Timer                 (void);

void USB_Edgeboard_Timr             (void);
void USB_Edgeboard_Handle           (void);
void USB_Edgeboard_TransmitKey      (uint16 time);

//���ܳ��Լ�
void USB_Edgeboard_ServoThreshold   (uint8 chanel);
void USB_Edgeboard_CarSpeed         (void);
void senddata_to_upper              (uint8_t addr,float senddata);
//==================================================UART ��������==================================================


#endif
