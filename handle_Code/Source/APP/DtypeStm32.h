#ifndef __DTYPE_STM32_H__
#define __DTYPE_STM32_H__

/* Data type definitions for STM32 MCU */

#ifndef TRUE
    #define TRUE                    (1 == 1)
    #define FALSE                   (1 == 0)
#endif

#ifndef NULL
    #define NULL                    ((void *)0)
#endif

#ifndef ON
    #define ON                      1
    #define OFF                     0
#endif

#if (!defined(UINT8)) 
   typedef unsigned char            UINT8;
#endif

#if (!defined(UINT16))
   typedef unsigned short             UINT16;
#endif

#if (!defined(UINT32))
   typedef unsigned int            UINT32;
#endif


#if (!defined(VUINT8)) 
   typedef volatile unsigned char   VUINT8;
#endif

#if (!defined(VUINT16))
   typedef volatile unsigned short    VUINT16;
#endif

#if (!defined(VUINT32))
   typedef volatile unsigned int   VUINT32;
#endif

#if (!defined(UINT8_PTR))
    typedef UINT8 *                 UINT8_PTR;
#endif

#if (!defined(UINT16_PTR))
    typedef UINT16 *                UINT16_PTR;
#endif

#if (!defined(UINT32_PTR))
    typedef UINT32 *                UINT32_PTR;
#endif

#if (!defined(BYTE)) 
   typedef unsigned char            BYTE;
#endif


#if (!defined(INT8U)) 
   typedef unsigned char            INT8U;
#endif

#if (!defined(INT8U)) 
   typedef unsigned char            BOOL;
#endif


#if (!defined(INT16U))
   typedef unsigned short             INT16U;
#endif

#if (!defined(u8)) 
   typedef unsigned char            u8;
#endif


#if (!defined(u16))
   typedef unsigned short             u16;
#endif

#if (!defined(u32))
   typedef unsigned int             u32;
#endif


#endif /* __DTYPE_STM32_H__ */

