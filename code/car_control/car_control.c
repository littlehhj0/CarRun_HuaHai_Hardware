/*
 * car_control.c
 *
 *  Created on: 2023��3��27��
 *      Author: wzl
 */

#include "voltage_sampling/voltage_sampling.h"
#include "car_control/car_control.h"
#include "motor/motor.h"
#include "servo/servo.h"
#include "uart/uart.h"
#include "key/key.h"

//����С�����Ʋ������Ľṹ��  ȫ�ֱ������ƺ��ģ�
IcarStruct icarStr;


//-------------------------------------------------------------------------------------------------------------------
// �������     ���ܳ�������ʼ��
// ����˵��     void
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void ICAR_Init(void)
{
    icarStr.Voltage = 0;                             //��ѹ
    icarStr.SpeedSet = 0.0f;                         //���Ŀ���ٶȣ�m/s
    icarStr.SpeedFeedback = 0.0f;                    //���ģ��ʵ���ٶȣ�m/s
    icarStr.ServoPwmSet = servoStr.thresholdMiddle;  //���ö����ʼ��pwm
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ���ܳ��ۺϴ����̼߳�����
// ����˵��     void
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void ICAR_Timer(void)
{
    //�����⵽�������£���ʼ�������ΰ������µ�ʱ����ֱ���ɿ�
    if(gpioStr.KeyPress)
    {
        icarStr.counterKeyA++;
    }
    else
    {
        //�ɿ�������������ΰ�������ʱ������100ms�����ı�С�����Ʋ����İ���״̬���а������¹������ݰ�����ʱ������������صĴ���
        if(icarStr.counterKeyA > 100)
            icarStr.keyPressed = true;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ���ܳ��ۺϴ�����
// ����˵��     void
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void ICAR_Handle(void)
{
    //�����̼߳�⣬�а������²���ʱ������100ms��������صĴ������
    if(icarStr.keyPressed)
    {
        //�ٴ�ȷ�ϰ���ʱ������ֹ����
        if(icarStr.counterKeyA > 100)
            //����λ�����Ͱ���ʱ����֪ͨ��λ�����Խ��з���
#if !USING_BLUETOOTH_OR_EGBOARD
            USB_Edgeboard_TransmitKey(icarStr.counterKeyA);
#endif

        //�����������ʱ������3�룬������ز���
        if(icarStr.counterKeyA > 3000 && !icarStr.selfcheckEnable)
        {
            // �Ƿ���õ�����������
            if(adc_sampling.key_contral == true)
            {
                adc_sampling.key_contral = false;
            }
            else
            {
                adc_sampling.key_contral = true;
            }
        }

        //��ɰ�����Ϣ�����������������Ϣ
        icarStr.counterKeyA = 0;
        icarStr.keyPressed = false;
        //��������Լ칦�ܣ����԰����û�
        if(icarStr.selfcheckEnable)
        {
            icarStr.errorCode &= ~(1<<8);
        }
    }

    //���ܳ��Լ칦��
    if(icarStr.selfcheckEnable)
    {
        //ICAR_Selfcheck();
    }
}
