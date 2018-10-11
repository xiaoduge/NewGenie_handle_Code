#ifndef _IWDG_DRIVER_H_
#define _IWDG_DRIVER_H_

/**
  ******************************************************************************
  * @file    IWDG_Driver.h 
  * @author  STM32 Team
  * @version V1.0.0
  * @date    10/10/2014
  * @brief   Interface for Watchdog access.
  ******************************************************************************
  *          DO NOT MODIFY ANY OF THE FOLLOWING CODE EXCEPT UNDER PERMISSION OF ITS PROPRIETOR
  * @copy
  *
  *
  * <h2><center>&copy; COPYRIGHT 2012 Shanghai ZhenNeng Corporation</center></h2>
  */ 

void IWDG_Init(void);

void IWDG_Feed(void);

#endif // _IWDG_DRIVER_H_
