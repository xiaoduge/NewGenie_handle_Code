/**
  ******************************************************************************
  * @file    IAP/inc/common.h 
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    10/15/2010
  * @brief   This file provides all the headers of the common functions.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _COMMON_H
#define _COMMON_H

#include "stm32f10x.h"

/* Includes ------------------------------------------------------------------*/
#include "hal_types.h"

#include "stm32f10x_flash.h"

/*** MACROS  ***/
#define ERROR_SUCCESS                   0x00
#define ERROR_FAILURE                   0x01
#define ERROR_INVALIDPARAMETER          0x02
#define ERROR_INVALID_TASK              0x03
#define ERROR_MSG_BUFFER_NOT_AVAIL      0x04
#define ERROR_INVALID_MSG_POINTER       0x05
#define ERROR_INVALID_EVENT_ID          0x06
#define ERROR_INVALID_INTERRUPT_ID      0x07
#define ERROR_NO_TIMER_AVAIL            0x08
#define ERROR_NV_ITEM_UNINIT            0x09
#define ERROR_NV_OPER_FAILED            0x0A
#define ERROR_INVALID_MEM_SIZE          0x0B
#define ERROR_NV_BAD_ITEM_LEN           0x0C

/* NV Items for system : begin*/

#define NV_SERIAL_ID (0x10)

#define calcSerialNvID(Idx)               ( Idx + NV_SERIAL_ID)

#define NV_CANID_ID                       ( 0X20) // fixed , do not change

#define NV_DEVID_ID                       ( 0x21) // fixed , do not change

#define NV_DTYPE_ID                       ( 0x22) // fixed , do not change

#define NV_CANID_SIZE                     (2)     // 2 bytes

#define NV_DEVID_SIZE                     (4)     // 4 bytes

#define NV_DTYPE_SIZE                     (8)     // 2 bytes


/* NV Items  for system : End */


/* Exported types ------------------------------------------------------------*/
typedef  void (*pFunction)(void);


#define ApplicationAddress    0x8003000

#if defined (STM32F10X_MD) || defined (STM32F10X_MD_VL)
 #define PAGE_SIZE                         (0x400)    /* 1 Kbyte */
 #define FLASH_SIZE                        (0x20000)  /* 64~128 KBytes */
#elif defined STM32F10X_CL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x40000)  /* 256 KBytes */
#elif defined STM32F10X_HD || defined (STM32F10X_HD_VL)
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x40000)  /* 256~512 KBytes */
#elif defined STM32F10X_XL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x100000) /* 1 MByte */
#else 
 #error "Please select first the STM32 device to be used (in stm32f10x.h)"    
#endif

#define HAL_ELEC_ID_SIZE                   (96/32*2)   // 6 * 16bits            

#define HAL_FLASH_BEGIN                    (0x08000000)

#define HAL_NV_PAGE_END                    ((FLASH_SIZE/PAGE_SIZE)-1) // YLF: WE GO HERE
#define HAL_NV_PAGE_CNT                    2 


#define HAL_FLASH_WORD_SIZE                4
#define HAL_FLASH_PAGE_SIZE                PAGE_SIZE


/* Compute the FLASH upload image size */  

#define FLASH_IMAGE_SIZE                   (uint32) (FLASH_SIZE - (ApplicationAddress - HAL_FLASH_BEGIN))

#define HAL_NV_PAGE_BEG                   ((HAL_NV_PAGE_END) - HAL_NV_PAGE_CNT + 1)

#define HAL_APP_PAGE_CNT                  ((FLASH_SIZE/PAGE_SIZE)-HAL_NV_PAGE_CNT-(ApplicationAddress - HAL_FLASH_BEGIN)/PAGE_SIZE)

/* ------------------------------------------------------------------------------------------------
 *                                         Constants
 * ------------------------------------------------------------------------------------------------
 */

#define HAL_SBL_IMG_BEG                (uint32)((ApplicationAddress - HAL_FLASH_BEGIN) / HAL_FLASH_WORD_SIZE)
#define HAL_SBL_IMG_CRC                (uint32)((ApplicationAddress - HAL_FLASH_BEGIN) / HAL_FLASH_WORD_SIZE)
#define HAL_SBL_IMG_END                (uint32)(HAL_NV_PAGE_BEG*PAGE_SIZE/HAL_FLASH_WORD_SIZE)

// HalFlashRead page & offset for the CRC and CRC-shadow.
//#define SBL_CRC_PAGE            (HAL_SBL_IMG_CRC * HAL_FLASH_WORD_SIZE / HAL_FLASH_PAGE_SIZE)
//#define SBL_CRC_OFFSET          (HAL_SBL_IMG_CRC * HAL_FLASH_WORD_SIZE % HAL_FLASH_PAGE_SIZE)
// FOR CAN INTERFACE
#define CAN_ADDRESS_MASK (0X3FF)

#define CAN_SRC_ADDRESS_POS (10) // BIT0~BIT9 in indentifer

#define CAN_SRC_ADDRESS(indenfier)((indenfier >> CAN_SRC_ADDRESS_POS) & CAN_ADDRESS_MASK)

#define CAN_DST_ADDRESS_POS (0) // BIT0~BIT9 in indentifer

#define CAN_DST_ADDRESS(indenfier)((indenfier >> CAN_DST_ADDRESS_POS) & CAN_ADDRESS_MASK)

#define CAN_BROAD_CAST_ADDRESS (0X3FF)

#define CAN_ADDRESS(indenfier)((indenfier) & CAN_ADDRESS_MASK)


//#define CAN_BUILD_HOST_IDENTIFIER(usIdentifier,usCanAdr)(usIdentifier=(1<<10)|(usCanAdr))

#define CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,usSrcCanAdr,usDstCanAdr)(ulIdentifier=(usSrcCanAdr<<CAN_SRC_ADDRESS_POS)|(usDstCanAdr << CAN_DST_ADDRESS_POS)|CAN_EFF_FLAG)



#define CAN_INVALID_ADDRESS (0XFFFF)

#define INVALID_CAN_ADDRESS(usAddress) ((usAddress & CAN_BROAD_CAST_ADDRESS) == CAN_BROAD_CAST_ADDRESS)


/* Exported functions ------------------------------------------------------- */

void HAL_SYSTEM_RESET(void);

void FLASH_DisableWriteProtectionPages(void);

void FLASH_Prepare(void);

uint32_t FLASH_PagesMask(__IO uint32_t Size);

uint8 checkRC(void);

void GetDeviceElecId(uint8 *id);

uint8 CmpDeviceElecId(uint8 *id);

uint16 HashDeviceElecId(void);

#endif  /* _COMMON_H */

/*******************(C)COPYRIGHT 2010 STMicroelectronics *****END OF FILE******/
