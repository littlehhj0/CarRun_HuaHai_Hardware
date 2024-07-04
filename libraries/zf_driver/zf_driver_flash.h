#ifndef _zf_driver_flash_h_
#define _zf_driver_flash_h_

#include "ifxFlash_cfg.h"
#include "zf_common_typedef.h"

#define EEPROM_BASE_ADDR             (IFXFLASH_DFLASH_START)
#define EEPROM_SIZE                  (IFXFLASH_DFLASH_SIZE)                     // һ����96KB
#define EEPROM_PAGE_SIZE             (EEPROM_SIZE / IFXFLASH_DFLASH_NUM_LOG_SECTORS)
#define EEPROM_PAGE_NUM              (IFXFLASH_DFLASH_NUM_LOG_SECTORS)          // 96KB��Ϊ��12ҳ

#define FLASH_DATA_SIZE              (IFXFLASH_DFLASH_PAGE_LENGTH)
#define EEPROM_PAGE_LENGTH           (EEPROM_PAGE_SIZE/FLASH_DATA_SIZE)         // ÿҳ���Դ�1024��uint32���͵�����


typedef union                                                                   // �̶������ݻ��嵥Ԫ��ʽ
{
    float   float_type;                                                         // float  ����
    uint32  uint32_type;                                                        // uint32 ����
    int32   int32_type;                                                         // int32  ����
    uint16  uint16_type;                                                        // uint16 ����
    int16   int16_type;                                                         // int16  ����
    uint8   uint8_type;                                                         // uint8  ����
    int8    int8_type;                                                          // int8   ����
}flash_data_union;                                                              // �����������ݹ���ͬһ�� 32bit ��ַ

extern flash_data_union flash_union_buffer[EEPROM_PAGE_LENGTH];

//====================================================FLASH ��������====================================================
uint8   flash_check                     (uint32 sector_num, uint32 page_num);                                   // У��FLASHҳ�Ƿ�������
void    flash_erase_page                (uint32 sector_num, uint32 page_num);                                   // ����ҳ
void    flash_read_page                 (uint32 sector_num, uint32 page_num, uint32 *buf, uint16 len);          // ��ȡһҳ
void    flash_write_page                (uint32 sector_num, uint32 page_num, const uint32 *buf, uint16 len);    // ���һҳ
void    flash_read_page_to_buffer       (uint32 sector_num, uint32 page_num);                                   // ��ָ�� FLASH ��ָ��ҳ���ȡ���ݵ�������
uint8   flash_write_page_from_buffer    (uint32 sector_num, uint32 page_num);                                   // ��ָ�� FLASH ��������ָ��ҳ��д�뻺����������
void    flash_buffer_clear              (void);                                                                 // ������ݻ�����
//====================================================FALSH ��������====================================================


#endif
