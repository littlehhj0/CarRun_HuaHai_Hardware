/*
 * key.h
 *
 *  Created on: 2023��3��28��
 *      Author: wzl
 */

#ifndef CODE_KEY_KEY_H_
#define CODE_KEY_KEY_H_

//����ͷ�ļ�
#include "cpu0_main.h"

//�궨��Ҫ��ʼ���ذ���
#define KEY_GPIO    484                 //p15_4

//���尴���ṹ��
typedef struct
{
    bool KeyPress;                      //��������
}GpioStruct;


//���������ṹ��
extern GpioStruct gpioStr;


//==================================================CAR_CONTROL ��������==================================================
void my_key_init            (void);
//==================================================CAR_CONTROL ��������==================================================

#endif
