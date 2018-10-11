#ifndef _TIME_SLICE_SCHEDULE_H_
#define _TIME_SLICE_SCHEDULE_H_

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


typedef void (* tss_schedule_handler)(void *arg);

typedef struct  tss_object{
  struct tss_object *next;
  tss_schedule_handler h;
  void *arg;
}tss_object;

void tss_init(int ts);

void tss_register(tss_schedule_handler h,void *arg,tss_object *obj);

void tss_unregister(tss_object *obj);

void tss_proc(void);

void tss_schedule(void);

#endif

