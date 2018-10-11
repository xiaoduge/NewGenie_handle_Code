/**
  ******************************************************************************
  * @file    hal_defs.h
  * @author  STM32 Team
  * @version V1.0.0
  * @date    10/10/2014
  * @brief   HAL Interface for MACROS.
  ******************************************************************************
  *          DO NOT MODIFY ANY OF THE FOLLOWING CODE EXCEPT UNDER PERMISSION OF ITS PROPRIETOR
  * @copy
  *
  *
  * <h2><center>&copy; COPYRIGHT 2012 Shanghai ZhenNeng Corporation</center></h2>
  */ 

#ifndef HAL_DEFS_H
#define HAL_DEFS_H


/* ------------------------------------------------------------------------------------------------
 *                                             Macros
 * ------------------------------------------------------------------------------------------------
 */

#define __LITTLE_ENDIAN 1234
#define __BIG_ENDIAN 4321

#define CPU_ENDIAN __LITTLE_ENDIAN

#ifndef BV
#define BV(n)      (1 << (n))
#endif

#ifndef BF
#define BF(x,b,s)  (((x) & (b)) >> (s))
#endif

#ifndef MIN
#define MIN(n,m)   (((n) < (m)) ? (n) : (m))
#endif

#ifndef MAX
#define MAX(n,m)   (((n) < (m)) ? (m) : (n))
#endif

#ifndef ABS
#define ABS(n)     (((n) < 0) ? -(n) : (n))
#endif


/* takes a byte out of a uint32 : var - uint32,  ByteNum - byte to take out (0 - 3) */
#define BREAK_UINT32( var, ByteNum ) \
          (uint8)((uint32)(((var) >>((ByteNum) * 8)) & 0x00FF))

#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
          ((uint32)((uint32)((Byte0) & 0x00FF) \
          + ((uint32)((Byte1) & 0x00FF) << 8) \
          + ((uint32)((Byte2) & 0x00FF) << 16) \
          + ((uint32)((Byte3) & 0x00FF) << 24)))

#define BUILD_UINT16(loByte, hiByte) \
          ((uint16)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

#define BUILD_UINT8(hiByte, loByte) \
          ((uint8)(((loByte) & 0x0F) + (((hiByte) & 0x0F) << 4)))

#define HI_UINT8(a) (((a) >> 4) & 0x0F)
#define LO_UINT8(a) ((a) & 0x0F)

#ifndef GET_BIT
#define GET_BIT(DISCS, IDX)  (((DISCS)[((IDX) / 8)] & BV((IDX) % 8)) ? TRUE : FALSE)
#endif
#ifndef SET_BIT
#define SET_BIT(DISCS, IDX)  (((DISCS)[((IDX) / 8)] |= BV((IDX) % 8)))
#endif
#ifndef CLR_BIT
#define CLR_BIT(DISCS, IDX)  (((DISCS)[((IDX) / 8)] &= (BV((IDX) % 8) ^ 0xFF)))
#endif


#define __swab16(x) ((uint16)(				\
	(((uint16)(x) & (uint16)0x00ffU) << 8) |			\
	(((uint16)(x) & (uint16)0xff00U) >> 8)))

#define __swab32(x) ((uint32)(				\
	(((uint32)(x) & (uint32)0x000000ffUL) << 24) |		\
	(((uint32)(x) & (uint32)0x0000ff00UL) <<  8) |		\
	(((uint32)(x) & (uint32)0x00ff0000UL) >>  8) |		\
	(((uint32)(x) & (uint32)0xff000000UL) >> 24)))


#if  CPU_ENDIAN == __LITTLE_ENDIAN
#define __cpu_to_be32(x) __swab32((x))
#define __be32_to_cpu(x) __swab32((x))
#define __cpu_to_be16(x) __swab16((x))
#define __be16_to_cpu(x) __swab16((x))

#else
#define __cpu_to_be32(x) ((uint32)(x))
#define __be32_to_cpu(x) ((uint32)(x))
#define __cpu_to_be16(x) ((uint16)(x))
#define __be16_to_cpu(x) ((uint16)(x))

#endif


#define htonl(x) (__cpu_to_be32(x))
#define htons(x) (__cpu_to_be16(x))

#define ntohl(x) (__be32_to_cpu(x))
#define ntohs(x) (__be16_to_cpu(x))

/*
 *  This macro is for use by other macros to form a fully valid C statement.
 *  Without this, the if/else conditionals could show unexpected behavior.
 *
 *  For example, use...
 *    #define SET_REGS()  st( ioreg1 = 0; ioreg2 = 0; )
 *  instead of ...
 *    #define SET_REGS()  { ioreg1 = 0; ioreg2 = 0; }
 *  or
 *    #define  SET_REGS()    ioreg1 = 0; ioreg2 = 0;
 *  The last macro would not behave as expected in the if/else construct.
 *  The second to last macro will cause a compiler error in certain uses
 *  of if/else construct
 *
 *  It is not necessary, or recommended, to use this macro where there is
 *  already a valid C statement.  For example, the following is redundant...
 *    #define CALL_FUNC()   st(  func();  )
 *  This should simply be...
 *    #define CALL_FUNC()   func()
 *
 * (The while condition below evaluates false without generating a
 *  constant-controlling-loop type of warning on most compilers.)
 */
#define st(x)      do { x } while (__LINE__ == -1)


/**************************************************************************************************
 */
#endif
