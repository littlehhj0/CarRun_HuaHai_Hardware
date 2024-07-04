#ifndef _zf_driver_timer_h_
#define _zf_driver_timer_h_

#include "zf_common_typedef.h"

//====================================================��ʱ�� ��������====================================================
void    system_start     (void);                                    // ��ʱ������
uint32  system_getval    (void);                                    // ��ʱ������ֵ��ȡ
//====================================================��ʱ�� ��������====================================================

//====================================================��ʱ�� ��չ����====================================================
#define system_getval_ms()          (system_getval() / 100000)      // ��ȡ��ǰ��ʱʱ��  ��λms
#define system_getval_us()          (system_getval() / 100   )      // ��ȡ��ǰ��ʱʱ��  ��λus
#define system_getval_ns()          (system_getval() * 10    )      // ��ȡ��ǰ��ʱʱ��  ��λns
//====================================================��ʱ�� ��չ����====================================================

#endif