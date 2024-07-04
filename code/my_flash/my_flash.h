/*
 * my_flash.h
 *
 *  Created on: 2023��3��30��
 *      Author: wzl
 */

#ifndef CODE_MY_FLASH_MY_FLASH_H_
#define CODE_MY_FLASH_MY_FLASH_H_


//����ͷ�ļ�
#include "cpu0_main.h"


//����ȫ�ֱ���
extern bool flashSaveEnable;
extern bool flashPIDEnable;


//====================================================MY_FLASH ��������====================================================
void my_flash_init              (void);
void my_flash_Handle            (void);
void my_flash_write_pid         (float* buf);
//====================================================MY_FLASH ��������====================================================


#endif
