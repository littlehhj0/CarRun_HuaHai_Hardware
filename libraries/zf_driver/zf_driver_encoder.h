#ifndef _zf_driver_encoder_h_
#define _zf_driver_encoder_h_

#include "zf_common_typedef.h"

// ��ö�ٶ��岻�����û��޸�
typedef enum // ö�ٱ���������
{
    TIM2_ENCODER_CH1_P00_7,      // T2��ʱ�� �������ſ�ѡ��Χ
    TIM2_ENCODER_CH1_P33_7,

    TIM3_ENCODER_CH1_P02_6,      // T3��ʱ�� �������ſ�ѡ��Χ

    TIM4_ENCODER_CH1_P02_8,      // T4��ʱ�� �������ſ�ѡ��Χ

    TIM5_ENCODER_CH1_P21_7,      // T5��ʱ�� �������ſ�ѡ��Χ
    TIM5_ENCODER_CH1_P10_3,

    TIM6_ENCODER_CH1_P20_3,      // T6��ʱ�� �������ſ�ѡ��Χ
    TIM6_ENCODER_CH1_P10_2,
}encoder_channel1_enum;

// ��ö�ٶ��岻�����û��޸�
typedef enum // ö�ٱ���������
{
    TIM2_ENCODER_CH2_P00_8,     // T2��ʱ�� �����������ſ�ѡ��Χ
    TIM2_ENCODER_CH2_P33_6,

    TIM3_ENCODER_CH2_P02_7,     // T3��ʱ�� �����������ſ�ѡ��Χ

    TIM4_ENCODER_CH2_P00_9,     // T4��ʱ�� �����������ſ�ѡ��Χ
    TIM4_ENCODER_CH2_P33_5,

    TIM5_ENCODER_CH2_P21_6,     // T5��ʱ�� �����������ſ�ѡ��Χ
    TIM5_ENCODER_CH2_P10_1,

    TIM6_ENCODER_CH2_P20_0,     // T6��ʱ�� �����������ſ�ѡ��Χ
}encoder_channel2_enum;

typedef enum  // ö�� ��ʱ�����
{
    TIM2_ENCODER,
    TIM3_ENCODER,
    TIM4_ENCODER,
    TIM5_ENCODER,
    TIM6_ENCODER,
}encoder_index_enum;

//====================================================������ ��������====================================================
int16 encoder_get_count     (encoder_index_enum encoder_n);
void  encoder_clear_count   (encoder_index_enum encoder_n);

void  encoder_quad_init     (encoder_index_enum encoder_n, encoder_channel1_enum count_pin, encoder_channel2_enum dir_pin);
void  encoder_dir_init      (encoder_index_enum encoder_n, encoder_channel1_enum ch1_pin, encoder_channel2_enum ch2_pin);
//====================================================������ ��������====================================================

#endif
