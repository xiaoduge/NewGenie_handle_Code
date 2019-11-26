/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                          (c) Copyright 2003-2006; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                            EXAMPLE CODE
*
*                                     ST Microelectronics STM32
*                                              with the
*                                   STM3210B-EVAL Evaluation Board
*
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : Brian Nagel
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

//#include <includes.h>
#define   DEF_FALSE 0
#define   DEF_TRUE  1

#include    <stdio.h>
#include    <string.h>
#include    <ctype.h>
#include    <stdlib.h>
#include    <stdarg.h>

#include    <ucos_ii.h>

#include    <cpu.h>
#include    <lib_def.h>
#include    <lib_mem.h>
#include    <lib_str.h>

#include    "stm32f10x_conf.h"

#include    <stdint.h>

#include    <app_cfg.h>
#include    <bsp.h>

#include "memory.h"
#include "task.h"
#include "timer.h"

#include "printf.h"

#include "UartCmd.h"

#include "serial.h"

#include "stm32_eval.h"


#include "Relay.h"

#include "DICA.h"

#include "Beep.h"

#if (IWDG_SUPPORT > 0)
#include "IWDG_Driver.h"
#endif

#if (RTC_SUPPORT > 0)
#include "RTC_Driver.h"
#endif

#include "osal_snv.h"

#include "sys_time.h"

#include "keyboard.h"

#include "common.h"

#include "sapp.h"

#include "display.h"

#include "LCD.h"

#if (CAN_SUPPORT > 0)
#include "CanCmd.h"
#endif


#if CYTMP_SUPPORT > 0
#include "cytma.h"
#endif

#include "zb.h"


/* ----------------- APPLICATION GLOBALS ------------------ */
static OS_STK AppTaskMainStk[APP_TASK_MAIN_STK_SIZE];
static OS_STK AppTaskRootStk[APP_TASK_ROOT_STK_SIZE];

//static  OS_EVENT       *AppUserIFMbox;

int ExTick;



/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

//static void AppTaskRoot(void *p_arg);


/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Arguments   : none
*
* Returns     : none
*********************************************************************************************************
*/

#if OS_APP_HOOKS_EN > 0

UINT32 ulAppTime = 0;
UINT32 ulAppTicks,jiffies = 0;
UINT32 ulAppSecond = 0;

void          App_TaskCreateHook    (OS_TCB          *ptcb)
{
}
void          App_TaskDelHook       (OS_TCB          *ptcb)
{
}
void          App_TaskIdleHook      (void)
{
    #if (IWDG_SUPPORT > 0)
    IWDG_Feed();
    #endif
    
}

void          App_TaskStatHook      (void)
{
}

#if OS_TASK_SW_HOOK_EN > 0
void          App_TaskSwHook        (void)
{
}
#endif

void          App_TCBInitHook       (OS_TCB          *ptcb)
{
}

void App_TimeTickHook(void)
{
    UINT8 ucLoop;

    ulAppTime++;
    if ( (OS_TICKS_PER_SEC / OS_TMR_CFG_TICKS_PER_SEC) == (ulAppTime))
    {
        // do someting
        ulAppTime = 0;
        jiffies++;
        ulAppTicks++;


        if (jiffies % OS_TMR_CFG_TICKS_PER_SEC == 0)
        {
            ulAppSecond++;
        }
        //if (0 == (jiffies % (OS_TMR_CFG_TICKS_PER_SEC/10)))
        //{
        //    CanCcbDeviceOutputPusleCheck();
        //}
        TimerProc();

        sys_time_proc();

        // do someting
        for (ucLoop = 0; ucLoop < SERIAL_MAX_NUMBER; ucLoop++)
        {
            Serial_CheckRcvIdleTime(ucLoop);
        }

       //DISPITF_CheckRcvIdleTime();
        
    }

}

void USER_Delay(int tick)
{
    int tend = ((int)jiffies) + tick;
    while(((int)jiffies) < tend);
}


#endif

void SecondTimer(void)
{
//    static int bToggle = 0;

    
    RunToggle();

    Disp_SecondTask();

    CanCcbSecondTask();

    zbSecondTask();
    
    CYTMA_second();

    //AlarmToggle();

    // print rtc time
#if (RTC_SUPPORT > 0)
    //RTC_Get();
#endif
    //    CheckPendingReport();
    //#if (USB_SUPPORT > 0) // for test purpose
    //{
    //    uint8_t temp[10];
    //    uint16_t len;
    //    len = USBCDC_recvdata(temp,10);
    //    if(len) USBCDC_senddata(temp,len);
    //}
    //#endif


}



UINT8 PidTimerProcess(Message *pMsg)
{
    UINT8 ucTimer = pMsg->msgHead.nMsgType;

    switch(ucTimer) /*MsgLen is userid */
    {
    case TIMER_CHECK_SECOND:
        SecondTimer();
        break;
    case TIMER_CHECK_KEY:
        {
            break;
        }
    case TIMER_CHECK_ALARM:
        Alarm(0);
        break;
    case TIMER_CHECK_BEEP:
        Beep(0);
        break;
    default:
        if (ucTimer >= TIMER_CCB_BEGIN && ucTimer < TIMER_CCB_END)
        {
            CanCcbTimeout(ucTimer - TIMER_CCB_BEGIN,(pMsg->msgHead.AddData & 0xffff));
        }
        break;
    }
    
    return 0;
}

UINT8 MainSappProc(Message *pMsg)
{
    UINT8 ucRet = TRUE;

    //MainAlarmWithDuration(1);               

    sappItfType = Interface_RS232;
    sappItfPort = pMsg->msgHead.MsgSeq;
    
    switch(sbRxBuf[RPC_POS_CMD0])
    {
    case RPC_SYS_APP:
        ucRet = SHZNAPP_SerialAppProc();
        break;
    case RPC_SYS_BOOT:
        ucRet = SHZNAPP_SerialBootProc();
        break;
    default:
        ucRet = SHZNAPP_SerialUnknowProc();
        break;
    }
    if (ucRet )
    {
        (void)SHZNAPP_SerialResp(sappItfPort);  // Send the SB response setup in the sbRxBuf passed to sblProc().
    }

    return 0;
}

#if USB_SUPPORT > 0
void USBCDC_CallBack(uint8_t ucPort,uint8_t event)
{
    if (0XFF == ucPort) ucPort = 1;
    
    if (event & (HAL_UART_RX_FULL|HAL_UART_RX_ABOUT_FULL|HAL_UART_RX_TIMEOUT))
    {
        SHZNAPP_SerialParse(ucPort);
    }
    
    if (event & HAL_UART_TX_EMPTY)
    {
    
        if (sappFlags & (1 << SAPP_CMD_RESET))
        {
            USBCDC_disconnect();    
            HAL_SYSTEM_RESET();
        }
        else if (sappFlags & (1 << SAPP_CMD_SYS_INIT))
        {
            USBCDC_disconnect();    
            HAL_SYSTEM_RESET();
        }
    }
}
#endif

void InitKeys(void)
{
    GPIO_KEY key;

    key.gpio = STM32F103_GPC(9);
    key.key = KEY_CODE_HAND_KEY;
    RegisterKey(&key);

    key.gpio = STM32F103_GPC(10);
    key.key = KEY_CODE_FOOT_KEY;
    RegisterKey(&key);
    

}
void  MainInit (void)
{

   VOS_SetLogLevel(VOS_LOG_DEBUG);
   UartCmdSetLogLevel(VOS_LOG_INFO);

   stm32_gpio_init();

   Ioctrl_input_init();

   InitRelays();

   RunInit(); // YLF: FOR RUN INDICATION
   
   Run(FALSE);

   AlarmInit();

   BeepInit();

   UartCmdInit();


#if (USB_SUPPORT > 0)
   USBCDC_init();
   USB_SetUsb2UartCallback(USBCDC_CallBack);
#endif

   //Printf_Init();

#if (OS_TASK_STAT_EN > 0)
    OSStatInit();                                               /* Determine CPU capacity                                   */
#endif

#if (RTC_SUPPORT > 0)
   RTC_Config();
   RTC_Get(); // init system time
#endif

   if (ERROR_SUCCESS != osal_snv_init())
   {
       VOS_LOG(VOS_LOG_ERROR,"Init SNV Error");
   }
   else
   {
       Config_Init();
   }


#if (CAN_SUPPORT > 0)
    CanCmdInit();
#endif

#if (I2C_SUPPORT > 0)

    gpI2cAdpater = hal_i2c_Init();

#endif

   KeyboardInit();

   InitKeys();

   LCD_Init();

#if TSC2007_SUPPORT > 0
   TSC2007_Init();
#endif

#if CYTMP_SUPPORT > 0
    CYTMA_Init();
#endif

   Disp_Init();

#if (IWDG_SUPPORT > 0)
   IWDG_Init();
#endif

   zbInit();

//   AppUserIFMbox = OSMboxCreate((void *)0);                  /* Create MBOX for communication between Kbd and UserIF     */
   //AppTaskCreate();                                                    /* Create application tasks                                 */

   // TIM_Init(TIMER1);

   // ylf: start timers
   AddTimer(TIMER_CHECK_SECOND,OS_TMR_OPT_PERIODIC,OS_TMR_CFG_TICKS_PER_SEC,0xffff);

   //AddTimer(TIMER_CHECK_KEY,OS_TMR_OPT_PERIODIC,10/(1000/OS_TMR_CFG_TICKS_PER_SEC),0xffff); // 100ms
   
   MainBeepWithDuration(1);

}


UINT8 PidSelfProcess(Message *pMsg)
{
    switch(pMsg->msgHead.nMsgType)
    {
    case SYSTEM_MSG_CODE_INIT:
        MainInit();
        break;
    case SYSTEM_MSG_CODE_DELAY:
        break;
    case SELF_MSG_CODE_USER_DISP:
        Disp_msg_Handler(pMsg);
        break;
    case SELF_MSG_CODE_USER_CANCMD:
        CanCmd_msg_Handler(pMsg);
        break;
        
    }
    return 0;
}

UINT8 PidKeyboardProcess(Message *pMsg)
{
    //VOS_LOG(VOS_LOG_DEBUG,"key %d,%d",pMsg->msgHead.AddData,pMsg->msgHead.nMsgType);
    Disp_KeyHandler(pMsg->msgHead.AddData,pMsg->msgHead.nMsgType);    
    return 0;
}

UINT8 PidTouchProcess(Message *pMsg)
{
    Disp_TouchHandler((TOUCH_EVENT *)pMsg->data);

    return 0;
}


void MainKickoff(void)
{
    Message *Msg;
    
    Msg = MessageAlloc(PID_SELF,0);
    
    if (Msg)
    {
        Msg->msgHead.nMsgType = SYSTEM_MSG_CODE_INIT;
        MessageSend(Msg);
    }

}


UINT8 Msg_proc(Message *pMsg)
{
    switch(pMsg->msgHead.nSendPid)
    {
    case PID_TIMER:
        return PidTimerProcess(pMsg);
    //case PID_SERIAL:
    //    return PidUartCmdProcess(pMsg);
#if (SERIAL_SUPPORT > 0)
    case PID_RS485:
        return PidSerialProcess(pMsg);
#endif      
#if (CAN_SUPPORT > 0)
    case PID_CAN:
        return PidCanProcess(pMsg);
#endif        
    case PID_SELF:
        return PidSelfProcess(pMsg);
    case PID_KB:
        return PidKeyboardProcess(pMsg);
    case PID_SAPP:
        return MainSappProc(pMsg);   
    case PID_CYTMA:
        return CYTMA_ItfProcess(pMsg); 
    case PID_TOUCH:
        return PidTouchProcess(pMsg);   
    default:
            break;
    }
    
    return 0;
}


void Main_Entry(Message *msg)
{
    Msg_proc(msg);

}


/*
*********************************************************************************************************
*                                      CREATE APPLICATION TASKS
*
* Description:  This function creates the application tasks.
*
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/

#if 0
static  void  AppTaskCreate(void)
{
  INT8U  err;

  OSTaskCreateExt(AppTaskUserIF,(void *)0,(OS_STK *)&AppTaskUserIFStk[APP_TASK_USER_IF_STK_SIZE-1],APP_TASK_USER_IF_PRIO,APP_TASK_USER_IF_PRIO,(OS_STK *)&AppTaskUserIFStk[0],
                    APP_TASK_USER_IF_STK_SIZE,
                    (void *)0,
                    OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);


#if (OS_TASK_NAME_SIZE > 8)
  OSTaskNameSet(APP_TASK_USER_IF_PRIO, "User I/F", &err);
#endif

}
#endif

/*
*********************************************************************************************************
*                                         USER INTERFACE TASK
*
* Description : This task updates the LCD screen based on messages passed to it by AppTaskCanCmd().
*
* Arguments   : p_arg   is the argument passed to 'AppStartUserIF()' by 'OSTaskCreate()'.
*
* Returns     : none
*********************************************************************************************************
*/
#if  0
static void AppTestFatFs(void)
{
      FATFS fs;
      FRESULT res;
      FIL file;
      char data[20];
      
    
      res = f_mount(0, &fs);
      if(res != FR_OK)
      {
          //INFO_Printf("f_mount fail %d\n",res);
      }
      else
      {
          res = f_open(&file, "0:/data.txt", FA_CREATE_ALWAYS | FA_WRITE);
          
          if(res != FR_OK)
          {
              //INFO_Printf("f_open fail %d\n",res);
          }
          else
          {
              f_puts("Test data ....",&file);    
      
              f_close(&file); 
    
              res = f_open(&file, "0:/data.txt", FA_READ);
              
              if(res != FR_OK)
              {
                  //INFO_Printf("f_open fail %d\n",res);
              }
              else
              {
                  if(f_gets(data, sizeof(data), &file)!=NULL)
                  {
                      //INFO_Printf(data);
                  }
          
                  f_close(&file); 
              }
          }
      
      }

}
#endif

static  void  AppTaskRoot (void *p_arg)
{
 
 (void)p_arg;

  BSP_Init(); 

  VOS_CreateTask(APP_TASK_MAIN_PRIO,(OS_STK *)&AppTaskMainStk[0],APP_TASK_MAIN_STK_SIZE,Main_Entry);

#if (OS_TASK_NAME_SIZE > 13)
  {
     INT8U err;
     OSTaskNameSet(APP_TASK_MAIN_PRIO, "Main-Task", &err);
  }
#endif

  MainKickoff();
 
  while(DEF_TRUE) 
  {
      OSTimeDlyHMSM(0,0,0,100);
  }
}


int  main (void)
{
    INT8U  err;

    CPU_IntDis();
    /* Set the Vector Table base location at 0x08000000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); 
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // ylf: 2bits for priority group 

    OSInit();                /* Initialize "uC/OS-II, The Real-Time Kernel"              */
   /* Create the start task */
   OSTaskCreateExt(AppTaskRoot,(void *)0,(OS_STK *)&AppTaskRootStk[APP_TASK_ROOT_STK_SIZE-1],APP_TASK_ROOT_PRIO,APP_TASK_ROOT_PRIO,(OS_STK *)&AppTaskRootStk[0],
                     APP_TASK_ROOT_STK_SIZE,
                     (void *)0,
                     OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);

#if (OS_TASK_NAME_SIZE > 13)
   OSTaskNameSet(APP_TASK_ROOT_PRIO, "Root-Task", &err);
#endif

    OSStart(); 
/* Start multitasking (i.e. give control to uC/OS-II)       */
}

