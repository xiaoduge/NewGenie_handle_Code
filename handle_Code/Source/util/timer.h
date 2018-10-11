#ifndef _TIMER_H_
#define _TIMER_H_

/**
  ******************************************************************************
  * @file    memory.h 
  * @author  STM32 Team
  * @version V1.0.0
  * @date    10/10/2014
  * @brief   Interface for timer access.
  ******************************************************************************
  * DO NOT MODIFY ANY OF THE FOLLOWING CODE EXCEPT UNDER PERMISSION OF ITS PROPRIETOR
  * @copy
  *
  *
  * <h2><center>&copy; COPYRIGHT 2012 Shanghai ZhenNeng Corporation</center></h2>
  */ 

#include <ucos_ii.h>
#include "DtypeStm32.h"


typedef struct
{
    UINT8       enType;
	UINT8       TimerId;
	UINT16      Para; 
	UINT16      TimerLen; 
	UINT32      dwTick; 
}TIMER;

#define MAX_TIMER_LEN 16


void InitTimer(void);

void AddTimer(UINT8 UsrId,UINT8 enType,UINT16 TimerLen,UINT16 Para);
void RmvTimer(UINT8 UserId);
void ModTimer(UINT8 UserId,UINT16 TimerLen,UINT16 usPara);
UINT8 CheckTimer(UINT8 UserId);

void TimerProc(void);
#endif
