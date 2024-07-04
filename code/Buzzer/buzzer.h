#ifndef CODE_BUZZER_BUZZER_H_
#define CODE_BUZZER_BUZZER_H_

//����ͷ�ļ�
#include "cpu0_main.h"


//��������������ṹ��
typedef struct
{
    bool Enable;                        //ʹ�ܱ�־
    uint16_t Times;                     //���д���
    uint16_t Counter;                   //������
    uint16_t Cut;                       //���ʱ��
    bool Silent;                        //�Ƿ���÷�����
}BuzzerStruct;


//��������Чö��
typedef enum
{
    BuzzerOk = 0,                       //ȷ����ʾ��
    BuzzerWarnning,                     //������ʾ��
    BuzzerSysStart,                     //������ʾ��
    BuzzerDing,                         //������һ��
    BuzzerFinish,                       //������ʾ��
    BuzzerVoltage,                      //���û�籨��
}BuzzerEnum;


//�����ṹ��
extern BuzzerStruct buzzerStr;


//==================================================buzzer ��������==================================================
void Buzzer_Init        (void);
void Buzzer_Timer       (void);
void Buzzer_Handle      (void);
void Buzzer_Enable      (BuzzerEnum buzzer);
//==================================================buzzer ��������==================================================

#endif
