/******************** (C) COPYRIGHT 2014 TM4Cxxx ********************
;* File Name          : Data_types.h
;* Author             : 张力阵
;* 数据类型预定义
*******************************************************************************/
/** @addtogroup Exported_types
  * @{
  */  
#include <stdint.h>
typedef int32_t  s32; 
typedef int16_t  s16;
typedef int8_t   s8;

typedef const int32_t sc32;        
typedef const int16_t sc16;        
typedef const int8_t  sc8;         

typedef volatile int32_t  vs32;
typedef volatile int16_t  vs16;
typedef volatile int8_t   vs8;

typedef volatile const int32_t vsc32;  
typedef volatile const int16_t vsc16;  
typedef volatile const int8_t vsc8;    

typedef uint32_t  u32;
typedef uint16_t  u16;
typedef uint8_t   u8;

typedef const uint32_t uc32;     
typedef const uint16_t uc16;     
typedef const uint8_t  uc8;      

typedef volatile uint32_t  vu32;
typedef volatile uint16_t  vu16;
typedef volatile uint8_t   vu8;

typedef volatile const uint32_t vuc32;  
typedef volatile const uint16_t vuc16;  
typedef volatile const uint8_t  vuc8;   

/* Basic Data Type Definition */
//typedef  uint8_t  BOOL;
typedef  uint8_t  uint8;
typedef  uint16_t uint16;
typedef  uint32_t uint32;
typedef  uint32_t uAddr;
typedef  uint32_t uBusWidth;

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif

#ifndef NULL
#define NULL    ((void *)0)
#endif

#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (1)
#endif
