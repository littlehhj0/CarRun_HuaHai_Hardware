#ifndef _zf_common_interrupt_h_
#define _zf_common_interrupt_h_

#include "zf_common_typedef.h"

void   interrupt_global_enable (uint32 primask);    // ȫ���ж�ʹ��
uint32 interrupt_global_disable (void);             // ȫ���ж�ʧ��
void   assert_interrupt_config  (void);             // �����ж�����

#endif

