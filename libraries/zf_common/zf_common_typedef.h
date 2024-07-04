#ifndef _zf_common_typedef_h_
#define _zf_common_typedef_h_

#include "math.h"
#include "stdio.h"
#include "stdint.h"
#include "stdarg.h"
#include "string.h"
#include "stdlib.h"
#include "ifx_types.h"
#include "PLATFORM_TYPES.H"
//=================================================== ���Ͷ��� ===================================================

//#define COMPATIBLE_WITH_OLDER_VERSIONS                                          // ���ݾɰ濪Դ��ӿ�
#define USE_ZF_TYPEDEF      (1)                                                 // �Ƿ��������Ͷ�������
#if USE_ZF_TYPEDEF
// ������������
// ����ʹ�� stdint.h ������������� �����ͻ ������Բü�
typedef signed char         int8;                                               // �з���  8 bits
typedef signed short int    int16;                                              // �з��� 16 bits
typedef signed int          int32;                                              // �з��� 32 bits
typedef signed long long    int64;                                              // �з��� 64 bits

typedef volatile uint8      vuint8;                                             // �ױ������� �޷���  8 bits
typedef volatile uint16     vuint16;                                            // �ױ������� �޷��� 16 bits
typedef volatile uint32     vuint32;                                            // �ױ������� �޷��� 32 bits
typedef volatile uint64     vuint64;                                            // �ױ������� �޷��� 64 bits

typedef volatile int8       vint8;                                              // �ױ������� �з���  8 bits
typedef volatile int16      vint16;                                             // �ױ������� �з��� 16 bits
typedef volatile int32      vint32;                                             // �ױ������� �з��� 32 bits
typedef volatile int64      vint64;                                             // �ױ������� �з��� 64 bits
#endif

#define ZF_ENABLE       (1)
#define ZF_DISABLE      (0)

#define ZF_TRUE         (1)
#define ZF_FALSE        (0)
//=================================================== ���Ͷ��� ===================================================

#endif
