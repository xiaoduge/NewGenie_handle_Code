#ifndef _SYS_TIME_H_
#define _SYS_TIME_H_

/**
  ******************************************************************************
  * @file    sys_time.h 
  * @author  STM32 Team
  * @version V1.0.0
  * @date    10/10/2014
  * @brief   Interface for timer access.
  ******************************************************************************
  *          DO NOT MODIFY ANY OF THE FOLLOWING CODE EXCEPT UNDER PERMISSION OF ITS PROPRIETOR
  * @copy
  *
  *
  * <h2><center>&copy; COPYRIGHT 2012 Shanghai ZhenNeng Corporation</center></h2>
  */ 

#include <ucos_ii.h>
#include "DtypeStm32.h"

#define SYS_TIMER_UNIT ((OS_TICKS_PER_SEC / OS_TMR_CFG_TICKS_PER_SEC)) // same as tick

enum
{
  SYS_TIMER_ONE_SHOT = 0,
  SYS_TIMER_PERIOD,
  SYS_TIMER_NUM,
};


typedef void (* sys_timeout_handler)(void *arg);

typedef struct  sys_timeo{
  struct sys_timeo *next;
  unsigned int time;
  sys_timeout_handler h;
  void *arg;
  unsigned int type; // refer SYS_TIMER_ONE_SHOT
  unsigned int period;
}sys_timeo;

void sys_inittime(void);
void sys_untimeout(sys_timeo *to);
void sys_timeout(unsigned int msecs, int type,unsigned int period,sys_timeout_handler h,void *arg,sys_timeo *timeout);
void sys_time_proc(void);
int sys_time_ticking(sys_timeo *to);

#endif
