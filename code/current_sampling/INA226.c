/*
 * INA226.c
 *
 *  Created on: 2023��7��28��
 *      Author: wzl
 */

#include "SysSe/Math/Ifx_LowPassPt1F32.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_delay.h"
#include "INA226.h"
#include "motor.h"

// �������IIC�����ṹ��-->���ڶԵ����ƽ������ݻ�ȡ
soft_iic_info_struct Soft_IIC_Struction;
// ����һ�׵�ͨ�˲����ṹ��
Ifx_LowPassPt1F32 LowPass_filter;
// ����һ�׵�ͨ�˲����Ĳ������ýṹ��
Ifx_LowPassPt1F32_Config LowPass_filter_config;


//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡina226����
// ����˵��     reg_addr            Ҫ��ȡ�ĵ�ַ
// ���ز���     reg_data            ��ȡ��������
//-------------------------------------------------------------------------------------------------------------------
int16 INA226_Read2Byte(uint8_t reg_addr)
{
    // ����������ݱ���
    int16 reg_data=0;
    bool cur_reg_sign_wdnmd = 0;

    // �����������洢����
    uint8_t data[2];

    // ��ȡ����
    soft_iic_read_8bit_registers(&Soft_IIC_Struction, reg_addr ,data ,2);

    // �ں�����
    reg_data= data[0];
    reg_data=(reg_data<<8)&0xFF00;
    reg_data |= data[1];

    //---New Added Start---
    //cur_reg_sign_wdnmd = (reg_data & 0x8000)>>15;
    //if (cur_reg_sign_wdnmd)
    //{
    //   reg_data = ~(reg_data - 1);
    //}
    //---New Added End---

    // ��������
    return reg_data;
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ��ina226д������
// ����˵��     reg_addr            д�����ݵĵ�ַ
// ����˵��     reg_data            д�������
// ���ز���     ����״̬��
//-------------------------------------------------------------------------------------------------------------------
uint8_t INA226_Write2Byte(uint8_t reg_addr,uint16 reg_data)
{
    // ��ȡuint16�ĸߵ�λ����
    uint8_t data_high=(uint8_t)((reg_data&0xFF00)>>8);
    uint8_t data_low=(uint8_t)reg_data&0x00FF;
    uint8_t data[2] = {data_high, data_low};

    // д������
    soft_iic_write_8bit_registers(&Soft_IIC_Struction, reg_addr, data, 2);
    system_delay_ms(2);

    // д�����
    return 1;
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ina226ģʽ��ʼ��
// ����˵��     void
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void INA226_Init(void)
{
    // ��ʼ��IIC�ӿ�
    soft_iic_init(&Soft_IIC_Struction, 0x40, 60, P33_12, P33_11);

    // ����ģʽ����
    while(!INA226_Write2Byte(Config_Reg, 0x424D));      // 0100_001_001_001_101��4��ƽ����0.204ms��0.204ms����������������ѹ
    while(!INA226_Write2Byte(Calib_Reg, 0x0800));       // �ֱ���Ϊ0.0005A����������Ϊ0.005��

    // ��ͨ�˲����ĳ�ʼ��
    LowPass_filter_config.cutOffFrequency = 10;                              // ��ͨ�˲����Ľ�ֹƵ��Ϊ10hz
    LowPass_filter_config.samplingTime = 0.0005;                             // �˲������ݲ���ʱ��Ϊ0.0005s
    LowPass_filter_config.gain = 0.85;                                       // ��ͨ�˲���������Ϊ0.85
    Ifx_LowPassPt1F32_init(&LowPass_filter, &LowPass_filter_config);         // ��ͨ�˲����ĳ�ʼ��
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ina226-->��ʱ��ȡ����
// ����˵��     void
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void INA226_Timer(void)
{
    // ��ȡ��������
    float current_motor = INA226_Read2Byte(Current_Reg) * 0.0005f / 5;  // �����Ĳ�����ѹ(mv) / ��������ֵ(m��)

    if(abs(current_motor) >= 4)
        return;

    motorStr.Current_motor = current_motor;
    // ����һ�׵�ͨ�˲�
    motorStr.Current_motor_After_filter = Ifx_LowPassPt1F32_do(&LowPass_filter, current_motor);
//    motorStr.Current_motor_After_filter = filterInput(&three_filter_for_current, current_motor);
}

