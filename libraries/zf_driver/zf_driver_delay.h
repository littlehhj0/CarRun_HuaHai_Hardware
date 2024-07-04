#ifndef _zf_driver_delay_h_
#define _zf_driver_delay_h_

#include "zf_common_typedef.h"


void    system_delay                (uint32 time, uint32 num);

//====================================================��ʱ ��������====================================================
#define system_delay_ms(time)       system_delay((1000000), (time))    // ������ʱʱ��  ��λms
#define system_delay_us(time)       system_delay((time*1000), (1))     // ������ʱʱ��  ��λus
#define system_delay_ns(time)       system_delay((time), (1))          // ������ʱʱ��  ��λns
//====================================================��ʱ ��������====================================================


#endif
