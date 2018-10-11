#ifndef _VOS_H_
#define _VOS_H_

/**
  ******************************************************************************
  * @file    vos.h 
  * @author  STM32 Team
  * @version V1.0.0
  * @date    10/10/2014
  * @brief   Interface for vos access.
  ******************************************************************************
  *          DO NOT MODIFY ANY OF THE FOLLOWING CODE EXCEPT UNDER PERMISSION OF ITS PROPRIETOR
  * @copy
  *
  *
  * <h2><center>&copy; COPYRIGHT 2012 Shanghai ZhenNeng Corporation</center></h2>
  */ 

#include "DtypeStm32.h"

//#define VOS_DEBUG
#include    <ucos_ii.h>


typedef OS_EVENT VOS_SEM;

int VOS_CreateSem(VOS_SEM **pSem,unsigned int value);
int VOS_SemP(VOS_SEM *pSem);
int VOS_SemV(VOS_SEM *pSem);
void VOS_DestroySem(VOS_SEM **pSem);

#endif
