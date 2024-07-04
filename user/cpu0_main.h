#ifndef CPU0_MAIN_H
#define CPU0_MAIN_H


//����ͷ�ļ�
#include "Cpu/Std/Ifx_Types.h"
#include "stdint.h"
#include "zf_common_headfile.h"

//���¶�������
typedef unsigned char  bool_t;
typedef float fp32;
typedef double fp64;


//app�ṹ��
typedef struct
{
    float32 sysFreq;                /**< \brief Actual SPB frequency */
    float32 cpuFreq;                /**< \brief Actual CPU frequency */
    float32 pllFreq;                /**< \brief Actual PLL frequency */
    float32 stmFreq;                /**< \brief Actual STM frequency */
} AppInfo;


typedef struct
{
    AppInfo info;                               /**< \brief Info object */
} App_Cpu0;


//�����壬����2�ֽڵ�����ת��
typedef union
{
    uint8           U8_Buff[2];
    uint16          U16;
    int16           S16;
}Byte2_Union;

//�����壬����4�ֽڵ�����ת��
typedef union
{
    uint8           U8_Buff[4];
    float           Float;
    unsigned long   U32;
}Byte4_Union;



IFX_EXTERN App_Cpu0 g_AppCpu0;

#endif
