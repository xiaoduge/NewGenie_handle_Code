/**
  ******************************************************************************
  * @file    hal_flash.h
  * @author  STM32 Team
  * @version V1.0.0
  * @date    10/10/2014
  * @brief   HAL Interface for FLASH access.
  ******************************************************************************
  *          DO NOT MODIFY ANY OF THE FOLLOWING CODE EXCEPT UNDER PERMISSION OF ITS PROPRIETOR
  * @copy
  *
  *
  * <h2><center>&copy; COPYRIGHT 2012 Shanghai ZhenNeng Corporation</center></h2>
  */ 

#ifndef HAL_FLASH_H
#define HAL_FLASH_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "common.h"
#include "hal_types.h"

/**************************************************************************************************
 * @fn          HalFlashRead
 *
 * @brief       This function reads 'cnt' bytes from the internal flash.
 *
 * input parameters
 *
 * @param       pg - Valid HAL flash page number (ie < 256).
 * @param       offset - Valid offset into the page (so < HAL_NV_PAGE_SIZE and byte-aligned is ok).
 * @param       buf - Valid buffer space at least as big as the 'cnt' parameter.
 * @param       cnt - Valid number of bytes to read: a read cannot cross into the next 32KB bank.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void HalFlashRead(uint16 pg, uint16 offset, uint8 *buf, uint16 cnt);

/**************************************************************************************************
 * @fn          HalFlashWrite
 *
 * @brief       This function reads 'cnt' bytes from the internal flash.
 *
 * input parameters
 *
 * @param       addr - Valid HAL flash write address: actual addr / 4 and quad-aligned.
 * @param       buf - Valid buffer space at least as big as the 'cnt' parameter.
 * @param       cnt - Valid number of bytes to write: a write cannot cross into the next 32KB bank.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
uint8 HalFlashWrite(uint32 addr, uint8 *buf, uint16 cnt);

/**************************************************************************************************
 * @fn          HalFlashErase
 *
 * @brief       This function erases 'cnt' pages of the internal flash.
 *
 * input parameters
 *
 * @param       pg - Valid HAL flash page number (ie < 128) to erase.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
uint8 HalFlashErase(uint16 pg);

#ifdef __cplusplus
};
#endif

#endif

/**************************************************************************************************
*/
