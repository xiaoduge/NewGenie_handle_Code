#ifndef _VOS_LOG_H_
#define _VOS_LOG_H_
/**
  ******************************************************************************
  * @file    voslog.h 
  * @author  VOS Team
  * @version V1.0.0
  * @date    10/10/2014
  * @brief   Interface for LOG access.
  ******************************************************************************
  *          DO NOT MODIFY ANY OF THE FOLLOWING CODE EXCEPT UNDER PERMISSION OF ITS PROPRIETOR
  * @copy
  *
  *
  * <h2><center>&copy; COPYRIGHT 2012 Shanghai ZhenNeng Corporation</center></h2>
  */ 


enum {
    VOS_LOG_ERROR    =  0,
    VOS_LOG_WARNING ,
    VOS_LOG_INFO  ,
    VOS_LOG_NOTICE ,
    VOS_LOG_DEBUG,
    VOS_LOG_BUTT,
};

#define VOS_LOG(level,fmt, args...)   VOS_logger(level, __FILE__, __LINE__,fmt,##  args)

void VOS_SetLogLevel(int iLevel);

int VOS_logger(int level,char *filename,int lineno,const char* fmt,...);

#endif

