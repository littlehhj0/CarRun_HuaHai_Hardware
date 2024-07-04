#include "IfxStm.h"
#include "IFXSTM_CFG.h"
#include "zf_driver_timer.h"

static uint32 systick_count[2];

//-------------------------------------------------------------------------------------------------------------------
//  �������      systick��ʱ������
//  ���ز���      void
//  ʹ��ʾ��      system_start(); // ������ʱ������¼�µ�ǰ��ʱ��
//-------------------------------------------------------------------------------------------------------------------
void system_start (void)
{
    systick_count[(IfxCpu_getCoreId())] = IfxStm_getLower(IfxStm_getAddress((IfxStm_Index)(IfxCpu_getCoreId())));
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ��õ�ǰSystem tick timer��ֵ
//  ���ز���     uint32           ���شӿ�ʼ�����ڵ�ʱ��(��λ10ns)
//  ʹ��ʾ��     uint32 tim = system_getval();
//  ��ע��Ϣ     �ں���0���ô˺�����ʹ��STM0ģ��  ����1��ʹ��STM1ģ��
//-------------------------------------------------------------------------------------------------------------------
uint32 system_getval (void)
{
    uint32 time;
    uint32 stm_clk;

    stm_clk = IfxStm_getFrequency(IfxStm_getAddress((IfxStm_Index)(IfxCpu_getCoreId())));

    time = IfxStm_getLower(IfxStm_getAddress((IfxStm_Index)(IfxCpu_getCoreId())));
    time = time - systick_count[(IfxCpu_getCoreId())];
    time = (uint32)((uint64)time * 100000000 / stm_clk);

    return time;
}




