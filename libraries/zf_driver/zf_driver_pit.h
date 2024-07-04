#ifndef _zf_driver_pit_h_
#define _zf_driver_pit_h_

#include "zf_common_typedef.h"

typedef enum // ö��ͨ����
{
    CCU60_CH0,
    CCU60_CH1,
    CCU61_CH0,
    CCU61_CH1,
}pit_index_enum;


#define pit_clear_flag(pit_index) (IfxCcu6_clearInterruptStatusFlag(IfxCcu6_getAddress((IfxCcu6_Index)(pit_index / 2)), (IfxCcu6_InterruptSource)(7+((pit_index % 2)*2))))

//====================================================PIT ��������====================================================
void pit_close               (pit_index_enum pit_index);
void pit_start               (pit_index_enum pit_index);
void pit_all_close           (void);
void pit_disable             (pit_index_enum pit_index);
void pit_enable              (pit_index_enum pit_index);
void pit_init                (pit_index_enum pit_index, uint32 time);
//====================================================PIT ��������====================================================

//====================================================PIT ��չ����====================================================
//-------------------------------------------------------------------------------------------------------------------
//  �������      pit_ms��ʼ��
//  ����˵��      pit_index           ѡ��CCU6ģ��
//  ����˵��      time                ����ʱ��(��λ������)
//  ���ز���      void
//  ʹ��ʾ��      pit_ms_init(CCU60_CH0, 5);      // ���������ж�5ms
//-------------------------------------------------------------------------------------------------------------------
#define pit_ms_init(pit_index, time)  pit_init((pit_index), (time*1000))   // (��λΪ ����)
//-------------------------------------------------------------------------------------------------------------------
//  �������      pit_us��ʼ��
//  ����˵��      pit_index           ѡ��CCU6ģ��
//  ����˵��      time                ����ʱ��(��λ��΢��)
//  ���ز���      void
//  ʹ��ʾ��      pit_us_init(CCU60_CH0, 5);      // ���������ж�5us
//-------------------------------------------------------------------------------------------------------------------
#define pit_us_init(pit_index, time)  pit_init((pit_index), (time))        // (��λΪ ΢��)
//====================================================PIT ��չ����====================================================

#endif
