#ifndef _TASK_H_
#define _TASK_H_

/**
  ******************************************************************************
  * @file    task.h 
  * @author  STM32 Team
  * @version V1.0.0
  * @date    10/10/2014
  * @brief   Interface for task access.
  ******************************************************************************
  *          DO NOT MODIFY ANY OF THE FOLLOWING CODE EXCEPT UNDER PERMISSION OF ITS PROPRIETOR
  * @copy
  *
  *
  * <h2><center>&copy; COPYRIGHT 2012 Shanghai ZhenNeng Corporation</center></h2>
  */ 

#include "msg.h"


#define MAX_PID_NUMBER 3

typedef void (*TaskEntry)(Message *msg);

typedef struct
{
    int Pid;
    MessageQueue *queue;
    TaskEntry entry;
}VOS_TASK;

typedef enum 
{
   PID_MAIN_TASK = 0, // user defined proc
   PID_DUMMY1_TASK,   // user defined proc
   PID_DUMMY2_TASK,   // user defined proc
   PID_NUM,
}TASK_PID;

#define    PID_TIMER  (MAX_PID_NUMBER+0)
#define    PID_RS485  (MAX_PID_NUMBER+1)
#define    PID_SERIAL (MAX_PID_NUMBER+2)
#define    PID_USB    (MAX_PID_NUMBER+3)
#define    PID_KB     (MAX_PID_NUMBER+4)
#define    PID_CAN    (MAX_PID_NUMBER+5)
#define    PID_ADC    (MAX_PID_NUMBER+6)
#define    PID_SELF   (MAX_PID_NUMBER+7)
#define    PID_SAPP   (MAX_PID_NUMBER+8)
#define    PID_TOUCH  (MAX_PID_NUMBER+9)
#define    PID_TSC2007 (MAX_PID_NUMBER+10)
#define    PID_CYTMA   (MAX_PID_NUMBER+11)

typedef enum
{
   // system defined
   SYSTEM_MSG_CODE_INIT = 0,
   SYSTEM_MSG_CODE_DELAY,
   SYSTEM_MSG_CODE_TSS,
}SYSTEM_MSG_CODE;

VOS_TASK *VOS_GetTaskByPid(int Pid);

void VOS_TaskInit(void);
VOS_TASK *VOS_CreateTask(int Pid,OS_STK  *ptos,INT32U   stk_size,TaskEntry entry);


#endif
