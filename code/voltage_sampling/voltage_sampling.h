/*
 * voltage_sampling.h
 *
 *  Created on: 2023��5��15��
 *      Author: wzl
 */


#ifndef CODE_VOLTAGE_SAMPLING_VOLTAGE_SAMPLING_H_
#define CODE_VOLTAGE_SAMPLING_VOLTAGE_SAMPLING_H_


#include "cpu0_main.h"
#include "zf_driver_adc.h"


//�궨����س���
#define VOLTAGE_VALUE   11.0
#define VOLTAGE_MIN     5.0
#define SAMPL_TIME      1000
#define CONVERSION_FAC  4095.0
#define MAX_VOLTAGE     3.3 * 4.0


//�����ѹ������صĽṹ��
typedef struct
{
    bool    if_adc_handle;     //adc�����־λ
    float   adc_getNum;        //adc����ֵ
    uint16  count;             //������
    bool    key_contral;       //�������Ƶ�������
}ADC_sampling;


//�����ṹ��
extern ADC_sampling adc_sampling;


//==================================================V_sampling ��������==================================================
void adc_Init       (void);
void adc_Timer      (void);
void adc_Handle     (void);
//==================================================V_sampling ��������==================================================

#endif
