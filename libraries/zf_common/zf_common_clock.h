#ifndef _zf_common_clock_h_
#define _zf_common_clock_h_

#include "zf_common_typedef.h"

#define AURIX_MCU_FREQUENCY         (200*1000*1000) // ���õ�Ƭ��Ƶ��Ϊ200M 200M(Ĭ��Ƶ��)

void set_clock              (void);                 // ����ϵͳƵ��
void get_clock              (void);                 // ��ȡϵͳƵ��
void disable_Watchdog       (void);                 // �رտ��Ź�
void clock_init             (void);                 // ʱ�ӳ�ʼ��
void cpu_wait_event_ready   (void);                 // �ȴ����к��ĳ�ʼ�����
#endif
