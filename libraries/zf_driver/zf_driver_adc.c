#include "Vadc/Adc/IfxVadc_Adc.h"
#include "zf_common_debug.h"
#include "zf_driver_adc.h"

#define ADC_SAMPLE_FREQUENCY	10000000 // ���10Mhz

uint8 adc_resolution[50];

//-------------------------------------------------------------------------------------------------------------------
// �������     ADC ת��һ��
// ����˵��     ch              ѡ�� ADC ͨ�� (��� zf_driver_adc.h ��ö�� adc_channel_enum ����)
// ���ز���     uint16          ת���� ADC ֵ
// ʹ��ʾ��     adc_convert(ADC1_CH0_A0);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint16 adc_convert (adc_channel_enum vadc_chn)
{
	Ifx_VADC_RES result;
	uint8 temp;
	do
	{
		result = IfxVadc_getResult(&MODULE_VADC.G[(vadc_chn / 16)], vadc_chn%16);
	} while(!result.B.VF);

	temp = 4 - (adc_resolution[vadc_chn] * 2);

	return((result.U&0x0fff)>>temp);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ADC ��ֵ�˲�ת��
// ����˵��     ch              ѡ�� ADC ͨ�� (��� zf_driver_adc.h ��ö�� adc_channel_enum ����)
// ����˵��     count           ��ֵ�˲�����
// ���ز���     uint16          ת���� ADC ֵ
// ʹ��ʾ��     adc_mean_filter_convert(ADC1_CH0_A0, 5);                        // �ɼ�5�� Ȼ�󷵻�ƽ��ֵ
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint16 adc_mean_filter_convert (adc_channel_enum vadc_chn, uint8 count)
{
    uint8 i;
    uint32 sum;

    zf_assert(count);// ���Դ�������Ϊ0

    sum = 0;
    for(i=0; i<count; i++)
    {
        sum += adc_convert(vadc_chn);
    }

    sum = sum/count;

    return (uint16)sum;
}

//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// �������     ADC ��ʼ��
// ����˵��     ch              ѡ�� ADC ͨ�� (��� zf_driver_adc.h ��ö�� adc_channel_enum ����)
// ����˵��     resolution      ѡ��ѡ��ͨ���ֱ���(���ͬһ�� ADC ģ���ʼ��ʱ�����˲�ͬ�ķֱ��� �����һ����ʼ���ķֱ�����Ч)
// ���ز���     void
// ʹ��ʾ��     adc_init(ADC1_CH0_A0, ADC_8BIT);                                // ��ʼ�� A0 Ϊ ADC1 �� channel0 ���빦�� �ֱ���Ϊ8λ
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void adc_init (adc_channel_enum vadc_chn, adc_resolution_enum resolution)
{
    static uint8 mudule_init_flag = 0;
    IfxVadc_Adc vadc;
    IfxVadc_Adc_Group adcGroup;
    IfxVadc_Adc_Config adcConfig;

    IfxVadc_Adc_initModuleConfig(&adcConfig, &MODULE_VADC);

    if(!mudule_init_flag)
    {
        mudule_init_flag = 1;
        IfxVadc_Adc_initModule(&vadc, &adcConfig);

    }
    else
    {
        vadc.vadc = adcConfig.vadc;
    }

    IfxVadc_Adc_GroupConfig adcGroupConfig;
    IfxVadc_Adc_initGroupConfig(&adcGroupConfig, &vadc);

    adcGroupConfig.groupId = (IfxVadc_GroupId)(vadc_chn / 16);
    adcGroupConfig.master  = adcGroupConfig.groupId;
    adcGroupConfig.arbiter.requestSlotBackgroundScanEnabled = TRUE;
    adcGroupConfig.backgroundScanRequest.autoBackgroundScanEnabled = TRUE;
    adcGroupConfig.backgroundScanRequest.triggerConfig.gatingMode = IfxVadc_GatingMode_always;
    adcGroupConfig.inputClass[0].resolution = IfxVadc_ChannelResolution_12bit;
    adcGroupConfig.inputClass[0].sampleTime = 1.0f/ADC_SAMPLE_FREQUENCY;
    adcGroupConfig.inputClass[1].resolution = IfxVadc_ChannelResolution_12bit;
    adcGroupConfig.inputClass[1].sampleTime = 1.0f/ADC_SAMPLE_FREQUENCY;

    IfxVadc_Adc_initGroup(&adcGroup, &adcGroupConfig);

    IfxVadc_Adc_ChannelConfig adcChannelConfig;
    IfxVadc_Adc_Channel       adcChannel;
    IfxVadc_Adc_initChannelConfig(&adcChannelConfig, &adcGroup);

    adcChannelConfig.channelId         = (IfxVadc_ChannelId)(vadc_chn%16);
    adcChannelConfig.resultRegister    = (IfxVadc_ChannelResult)(vadc_chn%16);
    adcChannelConfig.backgroundChannel = TRUE;

    IfxVadc_Adc_initChannel(&adcChannel, &adcChannelConfig);

    unsigned channels = (1 << adcChannelConfig.channelId);
    unsigned mask     = channels;
    IfxVadc_Adc_setBackgroundScan(&vadc, &adcGroup, channels, mask);

    IfxVadc_Adc_startBackgroundScan(&vadc);

    adc_resolution[vadc_chn] = resolution;
}

