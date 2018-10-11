#ifndef _MSG_H_
#define _MSG_H_

/**
  ******************************************************************************
  * @file    msg.h 
  * @author  STM32 Team
  * @version V1.0.0
  * @date    10/10/2014
  * @brief   Interface for message access.
  ******************************************************************************
  *          DO NOT MODIFY ANY OF THE FOLLOWING CODE EXCEPT UNDER PERMISSION OF ITS PROPRIETOR
  * @copy
  *
  *
  * <h2><center>&copy; COPYRIGHT 2012 Shanghai ZhenNeng Corporation</center></h2>
  */ 

#include    <ucos_ii.h>
#include "vos.h"

#define MAX_QUEUE_SIZE (32) // power of 2


typedef struct
{
    INT8U nSendPid;
    INT8U nRcvPid;
    INT8U nMsgType;
    INT8U MsgSeq;
    INT16U AddData;
    INT16U nMsgLen;
}MsgHead;


typedef struct
{
    MsgHead msgHead;
#ifdef VOS_DEBUG    
    char data[4+1]; // 4bytes for magic
#else
    char data[1]; 
#endif
}Message;

typedef struct
{

    Message *aMessages[MAX_QUEUE_SIZE];
    VOS_SEM  *sem;
    int mFront;
    int mRear;
    int mID;
    
}MessageQueue;

MessageQueue *createMessageQueue(int pid);
int queueMessage(MessageQueue *queue,Message *msg);

Message *waitMessage(MessageQueue *queue);
void MessageFree(Message *msg);
int MessageSend(Message *msg);
void MessageQueueDestroy(MessageQueue *queue);
Message *MessageAlloc(int pid,int MsgLen);

void MessageQueueInit(void);

#endif
