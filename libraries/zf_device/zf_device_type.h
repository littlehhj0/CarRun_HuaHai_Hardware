#ifndef _zf_device_type_h_
#define _zf_device_type_h_

#include "zf_common_debug.h"

//==============================================���� ���� ö����==================================================
typedef enum
{
    NO_CAMERE = 0,                                                              // ������ͷ
    CAMERA_BIN_IIC,                                                             // С��� IIC �汾
    CAMERA_BIN_UART,                                                            // С��� UART �汾
    CAMERA_GRAYSCALE,                                                           // �����
    CAMERA_COLOR,                                                               // ��ͫ
}camera_type_enum;

typedef enum
{
    NO_WIRELESS = 0,                                                            // ���豸
    WIRELESS_UART,                                                              // ���ߴ���
    BLUETOOTH_CH9141,                                                           // ���� CH9141
    GPS_TAU1201,
    WIFI_UART,
    WIFI_SPI,
}wireless_type_enum;
//==============================================���� ���� ö����==================================================

typedef void (*callback_function)(void);

//===========================================�����ص�����ָ�뼰��������==================================================
extern wireless_type_enum wireless_type;
extern callback_function wireless_module_uart_handler;                          // ���ߴ��ڽ����жϺ���ָ�룬���ݳ�ʼ��ʱ���õĺ���������ת

extern camera_type_enum camera_type;
extern callback_function camera_dma_handler;                                    // ����ͨѶ�жϺ���ָ�룬���ݳ�ʼ��ʱ���õĺ���������ת
extern callback_function camera_vsync_handler;                                  // ����ͨѶ�жϺ���ָ�룬���ݳ�ʼ��ʱ���õĺ���������ת
extern callback_function camera_uart_handler;                                   // ����ͨѶ�жϺ���ָ�룬���ݳ�ʼ��ʱ���õĺ���������ת
//===========================================�����ص�����ָ�뼰��������==================================================

//=============================================�жϻص� ��������===================================================
void   set_camera_type          (camera_type_enum type_set, callback_function vsync_callback, callback_function dma_callback, callback_function uart_callback);
void   set_wireless_type        (wireless_type_enum type_set, callback_function uart_callback);
//=============================================�жϻص� ��������===================================================

#endif
