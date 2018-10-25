#include    <ucos_ii.h>

#include    <cpu.h>
#include    <lib_def.h>
#include    <lib_mem.h>
#include    <lib_str.h>

#include    <string.h>

#include "stm32f10x.h"

#include "CanCmd.h"

#include "memory.h"
#include "msg.h"
#include "timer.h"

#include "stm32_eval.h"

#include "Can_driver.h"

#include "app_cfg.h"

#include "Config.h"

#include "Beep.h"

#include "Errorcode.h"

#include "common.h"

#include "sapp.h"

#include "cminterface.h"

#include "osal_snv.h"

#include "RTC_Driver.h"

#include "Display.h"

#include "Task.h"

#include "Relay.h"

#include "zb.h"

#define CAN_500kbps   (SystemCoreClock/2/8/500000)       
#define CAN_250kbps   (SystemCoreClock/2/8/250000)
#define CAN_125kbps   (SystemCoreClock/2/8/125000)   
#define CAN_62_5kbps  (SystemCoreClock/2/8/62500)   
#define CAN_31_25kbps (SystemCoreClock/2/8/31250)      

CAN_CCB  CanCcb[MAX_CAN_CCB_NUMBER];

CAN_Rcv_buff  CanRcvBuff[CAN_MAX_RCV_BUFFER];

CAN_Snd_buff  CanSndBuff[MAX_CAN_OUTPUT_BUFFER];

UINT8  ucSndbufAllocMask = 0;

UINT8  ucSndBufFront = 0;

UINT8  ucSndBufRear = 0;

UINT8  aSndBufQueue[MAX_CAN_OUTPUT_BUFFER];

UINT8  ucCanOutFlag = FALSE;

UINT8  ucCanBusySecond = 0;

UINT16 CanAddress = 0;

UINT16 CanHashAddr = 0;

UINT8 ucZombieCcbIndex;

CAN_Snd_Empty_CallBack gCanSndEmptyCB;

#define CAN_LOGIC_DEVICE_REGISETER_PERIOD (5000)// 30000ms

#define CAN_WAIT_RSP_TIEMR_LENGTH         (3000)// 30000ms

#define CAN_WAIT_TW_RSP_TIEMR_LENGTH      (10000)// 30000ms


#define APP_PROTOL_CHECK_HEART_BEAT(usDuration) ((usDuration != 0XFFFF) && (usDuration != 0))
#define APP_PROTOL_HEART_BEAT_CLIENT_PERIOD 20000
#define APP_PROTOL_HEART_BEAT_HOST_PERIOD   (APP_PROTOL_HEART_BEAT_CLIENT_PERIOD - 5000)

typedef struct
{
    MsgHead msgHead;
    void *para;
    void *para1;
}CANCMD_MSG;


#define CANCMD_MSG_LENGHT (sizeof(CANCMD_MSG)-sizeof(MsgHead))

typedef void (*cancmd_msg_cb)(void * para);

#define CAN_CMD_RSP_DELAY(ms) (CanAddress*ms)

const uint8_t aucCcb2TrxMap[] = {APP_TRX_CAN,APP_TRX_CAN,APP_TRX_ZIGBEE};
const uint8_t aucTrx2CcbMap[] = {CAN_LOCAL_CAN_INDEX,CAN_ZIGBEE_INDEX};

void CanCcbHeartBeatTimer(void *para);

UINT8 CanAllocSndBuff(void)
{
    UINT8 ucLoop ;

    for (ucLoop= 0; ucLoop < MAX_CAN_OUTPUT_BUFFER; ucLoop++)
    {
        if (ucSndbufAllocMask & (1 << ucLoop))
        {
            continue;
        }
        ucSndbufAllocMask |= (1 << ucLoop); 
        
        return ucLoop;
    }

    return MAX_CAN_OUTPUT_BUFFER; 
}

void CanFreeSndBuff(UINT8 ucIdx)
{
    if (ucIdx < MAX_CAN_OUTPUT_BUFFER)
    {
        ucSndbufAllocMask &= ~(1 << ucIdx); 
    }
}

UINT8 CanSndBufPop(void)
{
    UINT8 ucIdx ;

    if (ucSndBufRear == ucSndBufFront)
    {
        // queue full
        return MAX_CAN_OUTPUT_BUFFER;
    }
    
    ucIdx = aSndBufQueue[ucSndBufRear] ;
    
    ucSndBufRear = (ucSndBufRear +1)% MAX_CAN_OUTPUT_BUFFER ;

    return ucIdx;

}

UINT8 CanSndBufFull(void)
{
    return (((ucSndBufFront + 1) % MAX_CAN_OUTPUT_BUFFER )== ucSndBufRear);
}

UINT8 CanSndBufEmpty(void)
{
    return (ucSndBufFront == ucSndBufRear);
}

void CanResetTimer(void *para)
{
    HAL_SYSTEM_RESET();
}

uint8_t CanCcbIndex2Trx(uint8_t ucCcbIndex)
{
    return aucCcb2TrxMap[ucCcbIndex];
}

uint8_t Trx2CanCcbIndex(uint8_t ucIndex)
{
    return aucTrx2CcbMap[ucIndex];
}


/*********************************************************************
 * Function:        void CanCcbMsgSndCb(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Callback handler for sent CAN message 
 *
 * Note:            None.
 ********************************************************************/
void CanMsgSndCb(UINT8 ucIndex, UINT8 aucCmd[],UINT8 ucAddData)
{
    if ((sappFlags & (1 << SAPP_CMD_RESET))
        || (sappFlags & (1 << SAPP_CMD_SYS_INIT)))
    {
        sys_timeout(500,SYS_TIMER_ONE_SHOT,100,CanResetTimer,NULL,&CanCcb[ucIndex].to4Callback);
    }

    if (ucAddData) MainAlarmWithDuration(10);
}


/*********************************************************************
 * Function:        void SndCanData(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Send Can data
 *
 * Note:            None.
 ********************************************************************/
void SndCanData(void)
{
    UINT8 MsgLen;
    UINT8 ucTmp;
#if OS_CRITICAL_METHOD == 3         /* Allocate storage for CPU status register */
    OS_CPU_SR     cpu_sr = 0;
#endif      
    OS_ENTER_CRITICAL();

    if (!CanSndBufEmpty()) // there are something to be send
    {
        CAN_FRAME frame;
    
        ucTmp = aSndBufQueue[ucSndBufRear];

        
        if (CanSndBuff[ucTmp].data_len >= 8)
        {
            MsgLen = 8;
        }
        else
        {
            MsgLen = CanSndBuff[ucTmp].data_len;
        }        

        frame.can_dlc = MsgLen;

        CAN_BUILD_ADDRESS_IDENTIFIER(frame.can_id,CanSndBuff[ucTmp].usSrcCanAddress,CanSndBuff[ucTmp].usDstCanAddress);
        
        memcpy(frame.data,CanSndBuff[ucTmp].dat,MsgLen);
    
        if (STM32_CANSendMsgNoWait(&frame))
        {
            //MainBeepWithDuration(1);
        
            CanSndBuff[ucTmp].data_len -= MsgLen;

            if (0 == CanSndBuff[ucTmp].data_len)
            {
                // free buffer
                FreeMem(CanSndBuff[ucTmp].head);
            
                CanSndBufPop();  
        
                CanFreeSndBuff(ucTmp);

                CanMsgSndCb(CanSndBuff[ucTmp].ucCcbIndex,CanSndBuff[ucTmp].aucCmd,CanSndBuff[ucTmp].ucPort);
            }
            else
            {
                CanSndBuff[ucTmp].dat += MsgLen;
            }
            ucCanOutFlag = TRUE;
        }
    }
    else
    {
        ucCanOutFlag = FALSE;
        ucCanBusySecond = 0;
        
        CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE);
        
        if (gCanSndEmptyCB)(*gCanSndEmptyCB)();

    }
    OS_EXIT_CRITICAL();

}

void CanSndBufPush(UINT8 ucIdx)
{
    if (((ucSndBufFront + 1) % MAX_CAN_OUTPUT_BUFFER )== ucSndBufRear)
    {
        // queue full
        VOS_LOG(VOS_LOG_ERROR,"Unrecoverable bufffer error \r\n");
        
        return ;
    }
    aSndBufQueue[ucSndBufFront] = ucIdx;
    
    ucSndBufFront = (ucSndBufFront +1)% MAX_CAN_OUTPUT_BUFFER ;

    // 
    if (!ucCanOutFlag)
    {
        SndCanData();
    }

}


void CanCheckZombieCcb(void)
{

    CAN_CCB *pCcb = NULL;
    
    ucZombieCcbIndex = (ucZombieCcbIndex+1)%MAX_CAN_CCB_NUMBER;

    pCcb = &CanCcb[ucZombieCcbIndex];
    

    switch(CAN_ZIGBEE_INDEX)
    {
    case CAN_ZIGBEE_INDEX:
        if (pCcb->bit1Registered)
        {
            if (APP_PROTOL_CHECK_HEART_BEAT(pCcb->usHeartBeatPeriod))
            {
                if (!sys_time_ticking(&pCcb->to4HB))
                {
                    if (++pCcb->ucInactCnt >= 3)
                    {
                        VOS_LOG(VOS_LOG_ERROR,"Unrecoverable Error, Prepare to Reset \r\n");
        
                        CanResetTimer(pCcb);
                    }
                }
                else
                {
                    pCcb->ucInactCnt = 0;
                }
            }
        }
        break;
    }

    // current do nothing
}


void CanCcbInit(void)
{
    UINT8 ucLoop;
    for (ucLoop = 0; ucLoop < MAX_CAN_CCB_NUMBER; ucLoop++)
    {
        memset(&CanCcb[ucLoop],0,sizeof(CAN_CCB));
        CanCcb[ucLoop].ucCcbIdx = ucLoop;
    }

}


void CanTranceiveBufInit(void)
{
    UINT8 ucLoop;
    for (ucLoop = 0; ucLoop < CAN_MAX_RCV_BUFFER; ucLoop++)
    {
        CanRcvBuff[ucLoop].len = 0;
        CanRcvBuff[ucLoop].data_len = 0;
        CanRcvBuff[ucLoop].head = 0;
        CanRcvBuff[ucLoop].dat = 0;
    }

    ucSndbufAllocMask = 0;

    ucSndBufFront = 0;

    ucSndBufRear = 0;

    ucCanOutFlag = 0;

    ucCanBusySecond = 0;
    
}



void CanCmd_report(void *para,void *para1)
{
   Message *Msg;
   Msg = MessageAlloc(PID_SELF,CANCMD_MSG_LENGHT);

   if (Msg)
   {
       CANCMD_MSG *dmsg = (CANCMD_MSG *)Msg;
       dmsg->msgHead.nMsgType = SELF_MSG_CODE_USER_CANCMD;
       dmsg->para = para;
       dmsg->para1 = para1;
       MessageSend(Msg);
   }
}

void CanCmdInitizeCAN(void)
{
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;

    UINT32 dwCanId;
    UINT32 dwMask;
    
        /* CAN register init */
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);
    
    /* CAN cell init */  //  ylf: BaudRate = PCLK2/(CAN_Prescaler)/(BS1+BS2+1)
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = ENABLE;   // ylf: The transmit mailboxes as a transmit FIFO
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;// CAN_Mode_Normal; // CAN_Mode_LoopBack;
    CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1  = CAN_BS1_4tq; // ylf: PROP_SEG and PHASE_SEG1
    CAN_InitStructure.CAN_BS2  = CAN_BS2_3tq;  // ylf: PHASE_SEG2
    CAN_InitStructure.CAN_Prescaler = CAN_125kbps;
    
    
    STM_EVAL_CANInit(&CAN_InitStructure);

    
#ifdef CAN_ADR_FILTER    
    /* CAN filter init */

    dwMask = 0x1FFF;
    dwCanId = ((CanAddress & 0x3ff) << 3)|CAN_ID_EXT|CAN_RTR_DATA;
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (dwCanId >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterIdLow =  (dwCanId) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (dwMask >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = dwMask  & 0XFFFF; // least 3bits as net mask
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    CAN_FilterInitStructure.CAN_FilterNumber = 1;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (dwCanId >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterIdLow =  (dwCanId) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (dwMask >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = dwMask  & 0XFFFF; // least 3bits as net mask
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 1;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure); 


    dwCanId = ((0x3ffUL) << 3)|CAN_ID_EXT|CAN_RTR_DATA;
    CAN_FilterInitStructure.CAN_FilterNumber = 2;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (dwCanId >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterIdLow =  (dwCanId) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (dwMask >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = dwMask  & 0XFFFF; // least 3bits as net mask
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
    
    CAN_FilterInitStructure.CAN_FilterNumber = 3;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (dwCanId >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterIdLow =  (dwCanId) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (dwMask >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = dwMask  & 0XFFFF; // least 3bits as net mask
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 1;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure); 

    /* EXE BOARD */
    dwMask = 0x7FFFFF;
    dwCanId = (((APP_DEV_TYPE_EXE_BOARD << CAN_SRC_ADDRESS_POS )|APP_DEV_TYPE_MAIN_CTRL) << 3)|CAN_ID_EXT|CAN_RTR_DATA;
    CAN_FilterInitStructure.CAN_FilterNumber = 4;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (dwCanId >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterIdLow =  (dwCanId) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (dwMask >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = dwMask  & 0XFFFF; // least 3bits as net mask
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
    
    CAN_FilterInitStructure.CAN_FilterNumber = 5;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (dwCanId >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterIdLow =  (dwCanId) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (dwMask >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = dwMask  & 0XFFFF; // least 3bits as net mask
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 1;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);     

    /* FM BOARD */
    dwMask = 0x7FFFFF;
    dwCanId = (((APP_DEV_TYPE_FLOW_METER << CAN_SRC_ADDRESS_POS )|APP_DEV_TYPE_MAIN_CTRL) << 3)|CAN_ID_EXT|CAN_RTR_DATA;
    CAN_FilterInitStructure.CAN_FilterNumber = 6;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (dwCanId >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterIdLow =  (dwCanId) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (dwMask >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = dwMask  & 0XFFFF; // least 3bits as net mask
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
    
    CAN_FilterInitStructure.CAN_FilterNumber = 7;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (dwCanId >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterIdLow =  (dwCanId) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (dwMask >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = dwMask  & 0XFFFF; // least 3bits as net mask
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 1;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);         
#else
    /* CAN filter init */
    dwCanId = 0;
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (dwCanId >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterIdLow =  (dwCanId) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
    
    CAN_FilterInitStructure.CAN_FilterNumber = 1;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (dwCanId >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterIdLow =  (dwCanId) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 1;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure); 

#endif
    

    CAN_ITConfig(CAN1, CAN_IT_FMP0 /*| CAN_IT_FF0 | CAN_IT_FOV0*/, ENABLE);  // fifo0中断
    CAN_ITConfig(CAN1, CAN_IT_FMP1 /*| CAN_IT_FF1 | CAN_IT_FOV1*/, ENABLE);  // fifo1中断
    CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE);                              // 发送中断
    //CAN_ITConfig(CAN1, CAN_IT_EWG | CAN_IT_EPV | CAN_IT_BOF | CAN_IT_LEC 
    //            | CAN_IT_ERR | CAN_IT_WKU | CAN_IT_SLK, ENABLE);         // ERR中断

}


void CanCmdInit(void)
{

  // 1, retrive configurations
  if ( osal_snv_read( NV_CANID_ID, sizeof ( CanAddress ), &CanAddress ) != ERROR_SUCCESS )
  {
      //ucValidCfg = FALSE;
      CanAddress = APP_PROTOL_CANID_INVALID; /* leave to host to config*/
  }

  //memset(&to4Callback,0,sizeof(sys_timeo));

  //memset(&to4DelayExcu,0,sizeof(sys_timeo));

  ucZombieCcbIndex = 0;

  CanCcbInit();
  
  CanTranceiveBufInit();

  // 3. init hardware
  CanCmdInitizeCAN();

  gCanSndEmptyCB = NULL;
 
}


void CanCleanSndBuf()
{
    UINT8 ucIdx;

#if OS_CRITICAL_METHOD == 3         /* Allocate storage for CPU status register */
     OS_CPU_SR     cpu_sr = 0;
#endif      
    OS_ENTER_CRITICAL();

    do 
    {
        ucIdx = CanSndBufPop();
        if (ucIdx < MAX_CAN_OUTPUT_BUFFER)
        {
            // free buffer
            FreeMem(CanSndBuff[ucIdx].head);
            
            CanFreeSndBuff(ucIdx);
            
        }
    }while(ucIdx < MAX_CAN_OUTPUT_BUFFER);

    ucSndbufAllocMask = 0;
    ucSndBufRear = 0;
    ucSndBufFront = 0;
    ucCanOutFlag = FALSE;
    ucCanBusySecond = 0;
    
    OS_EXIT_CRITICAL();
    
}

void CanBusyCheck(void)
{
    if (ucSndbufAllocMask
        || ucCanOutFlag)
    {
       ucCanBusySecond++;
       if (ucCanBusySecond >= CAN_MAX_BUSY_SECOND)
       {
           // must be error, just reset CAN
           CanCleanSndBuf();
           
           CanCmdInitizeCAN();
       }
    }

    CanCheckZombieCcb();
}


/*********************************************************************
 * Function:        void RcvCanData(UINT8 ,UINT8 )
 *
 * PreCondition:    None
 *
 * Input:           Address Index, Can Link Packet Length
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:         CAN Network layer message reassemble
 *
 * Note:            None.
 ********************************************************************/
void RcvCanData(UINT8 ucIndex,CanRxMsg *pCanMsg )
{
   UINT16 usCanLen;
   
   if (0 == CanRcvBuff[ucIndex].len)
   {
       // judge message tag
       if (pCanMsg->Data[0] != RPC_UART_SOF)
       {
          // invalid tag
          return ;
       }

       // the head of networklayer have been received, this must be the second segment of networklayer
       usCanLen = pCanMsg->Data[1 + RPC_POS_LEN] + RPC_UART_FRAME_OVHD + RPC_FRAME_HDR_SZ;

       CanRcvBuff[ucIndex].len = usCanLen;

       CanRcvBuff[ucIndex].head = AllocMem(usCanLen);
       
       CanRcvBuff[ucIndex].dat = CanRcvBuff[ucIndex].head;

       if (CanRcvBuff[ucIndex].head)
       {
           memcpy(CanRcvBuff[ucIndex].dat,pCanMsg->Data,pCanMsg->DLC);
       }
       CanRcvBuff[ucIndex].dat += pCanMsg->DLC;

       CanRcvBuff[ucIndex].data_len = pCanMsg->DLC;

       CanRcvBuff[ucIndex].ucSubCanAdr = ucIndex;

       CanRcvBuff[ucIndex].usSrcAddr  = CAN_SRC_ADDRESS(pCanMsg->ExtId);
   }
   else
   {
       // other segment
       usCanLen = CanRcvBuff[ucIndex].data_len + pCanMsg->DLC;

       if (usCanLen > CanRcvBuff[ucIndex].len
          || ((usCanLen < CanRcvBuff[ucIndex].len)
              && (pCanMsg->DLC < 8)))
       {
           // invalid message
           CanRcvBuff[ucIndex].len = 0;

           // free buffer
           FreeMem(CanRcvBuff[ucIndex].head);

           CanRcvBuff[ucIndex].head = NULL;
           
           return;
       }
       if (CanRcvBuff[ucIndex].head)
       {
           memcpy(CanRcvBuff[ucIndex].dat,pCanMsg->Data,pCanMsg->DLC);
       } 
       CanRcvBuff[ucIndex].dat += pCanMsg->DLC;
       CanRcvBuff[ucIndex].data_len += pCanMsg->DLC;
       
   }


   if (CanRcvBuff[ucIndex].data_len == CanRcvBuff[ucIndex].len
    && 0 != CanRcvBuff[ucIndex].len)
   {
      // a frame is received

      if (CanRcvBuff[ucIndex].head)
      {
          CanRcvFrame(CAN_LOCAL_CAN_INDEX,ucIndex);
      }
      // clear rcv buffer
      CanRcvBuff[ucIndex].len = 0;
      CanRcvBuff[ucIndex].data_len = 0;
      if (CanRcvBuff[ucIndex].head)
      {
           FreeMem(CanRcvBuff[ucIndex].head);
      }
      CanRcvBuff[ucIndex].head = NULL;
      
   }
}

void CanCmdSetAdr(uint16_t usAddress)
{
    CanAddress = usAddress;

	Config_SetItem(NV_CANID_ID,NV_CANID_SIZE,&CanAddress);
}


UINT8 CanCmdIapSetAddr(uint8 *dat)
{
    // compare elecid
    int idx = 0;

	uint16_t usTmpAdr;
    
    uint8 *data = &dat[RPC_POS_DAT0];
    
    if (data[idx] != (HAL_ELEC_ID_SIZE << 1))
    {
        return FALSE; // invalid message
    }
    
    idx += 1;
    
    if (0 != CmpDeviceElecId(&data[idx]))
    {
        return FALSE; // not target to us
    }
    
    {
        idx += (HAL_ELEC_ID_SIZE << 1);
    
        usTmpAdr = (data[idx] << 8)|data[idx+1];//BUILD_UINT16(data[idx+1],data[idx]) ;
    
        usTmpAdr &= 0x7ff;

        CanCmdSetAdr(usTmpAdr);

        CanCmdInitizeCAN();
    }
    return TRUE;

}



void CanCcbSetOnLineFlag(uint8_t ucCcbIdx,uint8_t bFlag)
{
    CanCcb[ucCcbIdx].bit1OnlineState = bFlag;
}

/* Callback */

UINT8 PidCanProcess(Message *pMsg)
{
    UINT8 ucTmp;

    UINT32 Identifier;

    UINT16 usSrc,usDst;
    
    CanRxMsg *pCanMsg = (CanRxMsg *)pMsg->data;

    Identifier = (UINT32)pCanMsg->ExtId;

    usDst = CAN_DST_ADDRESS(Identifier);
    usSrc = CAN_SRC_ADDRESS(Identifier);

    if (CAN_BROAD_CAST_ADDRESS == usDst)
    {
        ucTmp = CAN_RCV_BUFFER_COMMON_INDEX ;
    }
    else if (CanAddress == usDst)
    {
        ucTmp = CAN_RCV_BUFFER_LOCAL_INDEX ;
    }  
    else if ((usDst == APP_DEV_TYPE_MAIN_CTRL) 
          && (usSrc == APP_DEV_TYPE_EXE_BOARD))
    {
        ucTmp = CAN_RCV_BUFFER_LOCAL_INDEX + 1;
    }
    else if ((usDst == APP_DEV_TYPE_MAIN_CTRL) 
          && (usSrc == APP_DEV_TYPE_FLOW_METER))
    {
        ucTmp = CAN_RCV_BUFFER_LOCAL_INDEX + 2;
    }
    else
    {
        return 0;
    }

    RcvCanData(ucTmp,pCanMsg);

    return 0;
}



/**************************************************************************************************
 * @fn          SHZNAPP_CanResp
 *
 * @brief       Make the SB response.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the downloaded code has been enabled; FALSE otherwise.
 */
uint8 SHZNAPP_CanResp(uint16 usDstCanAdr,uint16 usSrcCanAdr,UINT8 ucAddData)
{
  uint8 fcs = 0, len = sbTxBuf[RPC_POS_LEN] + RPC_FRAME_HDR_SZ;
  uint8 rtrn = FALSE;
  uint8 idx;

  if (INVALID_CAN_ADDRESS(usSrcCanAdr))
  {
      return rtrn;
  }
  
  for ( idx = RPC_POS_LEN; idx < len; idx++)
  {
    fcs ^= sbTxBuf[idx];
  }
  sbTxBuf[len] = fcs;


  sbTxBuf[-1] = RPC_UART_SOF;

  {
    UINT8 ucIdx ;

    UINT8 *pData;

    pData = AllocMem(len + 2); // 1byte for SOF ,1byte for checksum

    if (!pData)
    {
        return rtrn;
    }

    // send ack to network controller
    ucIdx = CanAllocSndBuff();

    if(ucIdx >= MAX_CAN_OUTPUT_BUFFER)
    {
        // no buffer
        FreeMem(pData);
        return rtrn;
    }
    
    CanSndBuff[ucIdx].head         = pData;
    CanSndBuff[ucIdx].dat          = pData; // init to pointer to begin of data area
    CanSndBuff[ucIdx].ucCcbIndex   = 0;
    CanSndBuff[ucIdx].len          = len + 2; 
    CanSndBuff[ucIdx].data_len     = CanSndBuff[ucIdx].len;            
    CanSndBuff[ucIdx].ucCanAdrType = 0;
    CanSndBuff[ucIdx].usSrcCanAddress = usSrcCanAdr;
    CanSndBuff[ucIdx].usDstCanAddress = usDstCanAdr;
    CanSndBuff[ucIdx].ucPort          = ucAddData;

    CanSndBuff[ucIdx].aucCmd[0] = sbTxBuf[RPC_POS_CMD0];
    CanSndBuff[ucIdx].aucCmd[1] = sbTxBuf[RPC_POS_CMD1];

    memcpy(pData,sbTxBuf-1,len + 2);
    
    // push to queue
    CanSndBufPush(ucIdx);
  }
  
  return TRUE;
}


void CanCcbHeartBeatTimer_msg_handler(void *para)
{
    CAN_CCB *pCcb = (CAN_CCB *)para;
    
    if (pCcb->bit3HeartBeat)
    {
        pCcb->bit3HeartBeat--;

        if (pCcb->bit3HeartBeat)
        {
            sys_timeout(pCcb->usHeartBeatPeriod,SYS_TIMER_ONE_SHOT,pCcb->usHeartBeatPeriod,CanCcbHeartBeatTimer,pCcb,&pCcb->to4HB);
            return ;
        }
    }

    VOS_LOG(VOS_LOG_DEBUG,"lost heartbeat\r\n");

    pCcb->bit1Registered = FALSE;
    pCcb->ucMachineState = MACHINE_STATE_IDLE;

    // lost heartbeat
    CanCcbSetOnLineFlag(pCcb->ucCcbIdx,FALSE);

    Disp_CanItfNotify(pCcb->ucCcbIdx,DISP_CANITF_CODE_LOST_HB);
    
}


void CanCcbHeartBeatTimer(void *para)
{
    CanCmd_report(CanCcbHeartBeatTimer_msg_handler,para);

}

void CanCcbTimeout(UINT8 ucIndex,UINT16 usPara)
{
    uint8_t ucTmp = CanCcb[ucIndex].ucMachineState;
    
    CanCcb[ucIndex].ucMachineState = MACHINE_STATE_IDLE;

    switch(ucTmp)
    {
    case MACHINE_STATE_MAIN_WAIT_OPRATION_RSP:
        Disp_DisplayHandleOpsTimeout((usPara&0XFF));
        break;
    case MACHINE_STATE_MAIN_WAIT_REGISTER_RSP:
        break;
    default:
        break;
    }

    VOS_LOG(VOS_LOG_WARNING,"CanCcbTimeout %d\r\n",ucTmp);
}


UINT16 CanCmdHashAdr(void)
{
    // calc hash address
    UINT16 usCanHashAddr;
    usCanHashAddr = (HashDeviceElecId())|(SysTick->VAL & 0xffff);
    
    srand(usCanHashAddr);
    
    usCanHashAddr = APP_PROTOL_CANID_DYNAMIC_BEGIN + (rand() % APP_PROTOL_CANID_DYNAMIC_RANGE);

    return usCanHashAddr;
}



void CanCcbAfDataClientMsgCnfCommProc(UINT8 ucCcbIndex)
{

    CanCcb[ucCcbIndex].bit3HeartBeat   = APP_EXE_HB_NUM;

    CanCcbSetOnLineFlag(ucCcbIndex,TRUE);


}

uint8 CanCcbAfDataClientOnLineNotiCnfMsg(UINT8 ucCcbIndex,uint8_t *msg)
{

    APP_PACKET_ONLINE_NOTI_CONF_STRU *pmg = (APP_PACKET_ONLINE_NOTI_CONF_STRU *)msg; 
    APP_PACKET_ONLINE_NOTI_CONF_HANDLER_STRU *pInfo = (APP_PACKET_ONLINE_NOTI_CONF_HANDLER_STRU *)pmg->aucInfo;


    CanCcb[ucCcbIndex].ucMachineState    = MACHINE_STATE_IDLE;

    CanCcb[ucCcbIndex].usHeartBeatPeriod = pInfo->usHeartBeatPeriod;

    CanCcb[ucCcbIndex].bit1Registered    = TRUE;

    RmvTimer(TIMER_CCB_BEGIN + ucCcbIndex);

    VOS_LOG(VOS_LOG_DEBUG,"hbp %d\r\n",CanCcb[ucCcbIndex].usHeartBeatPeriod);

    if (APP_PROTOL_CHECK_HEART_BEAT(CanCcb[ucCcbIndex].usHeartBeatPeriod))
    {
        //start time
        sys_timeout(CanCcb[ucCcbIndex].usHeartBeatPeriod,SYS_TIMER_ONE_SHOT,CanCcb[ucCcbIndex].usHeartBeatPeriod,CanCcbHeartBeatTimer,&CanCcb[ucCcbIndex],&CanCcb[ucCcbIndex].to4HB);
    }
    else
    {
        sys_untimeout(&CanCcb[ucCcbIndex].to4HB);
    }    

    CanCcbAfDataClientMsgCnfCommProc(ucCcbIndex);

    Disp_DisplayHandleIndConf(CanCcbIndex2Trx(ucCcbIndex),pmg);

    CanCcb[ucCcbIndex].ucRegTryNum = 0;

    return 0;
}

void CanCcbHeartBeat_msg_handler(void *para)
{
    UINT8 ucCcbIndex = (UINT8)(UINT32)para;
    UINT8 ucTransId  = (((UINT32)para) >> 8)& 0XFF;

    APP_PACKET_HEART_BEAT_RSP_STRU tsr;

    tsr.hdr.ucLen       = APP_POROTOL_PACKET_HEART_BEAT_RSP_PAYLOAD_LENGTH;
    tsr.hdr.ucMsgType   = APP_PACKET_COMM_HEART_BEAT|APP_PROTOL_PACKET_RSP_MASK;
    tsr.hdr.ucTransId   = ucTransId;
    tsr.hdr.ucDevType   = APP_DEV_TYPE_HAND_SET;

    /*Collect src Address here since it may be changed by other proc */
    CanCcb[ucCcbIndex].usSrcAddr = APP_DEV_TYPE_MAIN_CTRL;
 
    (void)CanSndSappCmd(ucCcbIndex,SAPP_CMD_DATA,0,(uint8_t *)&tsr,APP_POROTOL_PACKET_HEART_BEAT_RSP_TOTAL_LENGTH);
    
}

void CanCcbHeartBeat(void *para)
{
     CanCmd_report(CanCcbHeartBeat_msg_handler,para);
}

uint8_t CanCcbAfDataClientHeartBeatMsg(UINT8 ucCcbIndex,uint8_t *msg)
{

    APP_PACKET_HEART_BEAT_REQ_STRU *pmg = (APP_PACKET_HEART_BEAT_REQ_STRU *)msg; 

    APP_PACKET_HO_STATE_STRU *pHoState  = (APP_PACKET_HO_STATE_STRU *)pmg->aucData;
    
    if (APP_PROTOL_CHECK_HEART_BEAT(CanCcb[ucCcbIndex].usHeartBeatPeriod))
    {
        sys_timeout(CanCcb[ucCcbIndex].usHeartBeatPeriod,SYS_TIMER_ONE_SHOT,CanCcb[ucCcbIndex].usHeartBeatPeriod,CanCcbHeartBeatTimer,&CanCcb[ucCcbIndex],&CanCcb[ucCcbIndex].to4HB);
    }
    
    Disp_DisplayHeartBeatNotify(pHoState);    
    
    VOS_LOG(VOS_LOG_DEBUG,"rcv heartbeat : %d\r\n",ucCcbIndex);

    sys_timeout(CAN_CMD_RSP_DELAY(20),SYS_TIMER_ONE_SHOT,CAN_CMD_RSP_DELAY(20),CanCcbHeartBeat,(void *)(ucCcbIndex|(pmg->hdr.ucTransId << 8)),&CanCcb[ucCcbIndex].to4DelayExcu);

    CanCcbAfDataClientMsgCnfCommProc(ucCcbIndex);

    return 0;
}


uint8 CanCcbAfDataClientClientHostResetMsg(UINT8 ucCcbIndex,uint8_t *msg)
{

    VOS_LOG(VOS_LOG_DEBUG,"rcv reset : %d\r\n",ucCcbIndex);

    CanCcb[ucCcbIndex].bit1OnlineState = 0;

	CanCcb[ucCcbIndex].bit1Registered = 0;

    CanCcb[ucCcbIndex].ucMachineState = MACHINE_STATE_IDLE;

    return 0;
}

void CanCcbResetRegisterFlag(UINT8 ucCcbIndex)
{
	CanCcb[ucCcbIndex].bit1Registered = 0;
    CanCcb[ucCcbIndex].ucMachineState = MACHINE_STATE_IDLE;
}


uint8 CanCcbAfDataHandleOpsMsg(UINT8 ucCcbIndex,uint8_t *msg)
{
    APP_PACKET_HO_STRU *pOpsMsg = (APP_PACKET_HO_STRU *)msg; 
    
    /* 临时措施: ZIGBEE 总认为是专用的 : 2018/04/11 */
    if (CAN_ZIGBEE_INDEX == ucCcbIndex)
    {
        CanCcb[ucCcbIndex].ucDedicated = TRUE;
    }

    CanCcb[ucCcbIndex].bit3HeartBeat   = APP_EXE_HB_NUM;
    
    RmvTimer(TIMER_CCB_BEGIN + ucCcbIndex);

    CanCcb[ucCcbIndex].ucMachineState = MACHINE_STATE_IDLE;

    Disp_DisplayHandleOpsEntry(CanCcbIndex2Trx(ucCcbIndex),pOpsMsg,CanCcb[ucCcbIndex].ucDedicated);
        
    return 0;
}

uint8 CanCcbAfDataClientReportMsg(UINT8 ucCcbIndex,uint8_t *msg)
{
    APP_PACKET_CLIENT_RPT_IND_STRU *pMsg = (APP_PACKET_CLIENT_RPT_IND_STRU *)msg; 

    Disp_ClientReportEntry(pMsg);  
    
    return 0;
}


uint8 CanCcbAfDataMsg(UINT8 ucCcbIndex)
{
    APP_PACKET_COMM_STRU *pmg = (APP_PACKET_COMM_STRU *)&sbRxBuf[RPC_POS_DAT0]; 
    switch((pmg->ucMsgType & 0x7f))
    {
    case APP_PAKCET_COMM_ONLINE_NOTI:
        if ((pmg->ucMsgType & 0x80))
        {
            return CanCcbAfDataClientOnLineNotiCnfMsg(ucCcbIndex,(uint8_t *)pmg);
        }
        break;
    case APP_PACKET_COMM_HEART_BEAT:
        if ((!(pmg->ucMsgType & 0x80)))
        {
            return CanCcbAfDataClientHeartBeatMsg(ucCcbIndex,(uint8_t *)pmg);
        }
        break;
    case APP_PACKET_COMM_HOST_RESET:
        if ((!(pmg->ucMsgType & 0x80)))
        {
            return CanCcbAfDataClientClientHostResetMsg(ucCcbIndex,(uint8_t *)pmg);
        }
        break;
    case APP_PACKET_HAND_OPERATION:
        {
            return CanCcbAfDataHandleOpsMsg(ucCcbIndex,(uint8_t *)pmg);
        }
    case APP_PACKET_CLIENT_REPORT: /* report as broadcast messge */
        if (!(pmg->ucMsgType & 0x80))
        {
            return CanCcbAfDataClientReportMsg(ucCcbIndex,(uint8_t *)pmg);
        }
        break;
    default: // yet to be implemented
        VOS_LOG(VOS_LOG_WARNING,"unknow msg type %d\r\n",pmg->ucMsgType);
        break;
    }

    return 0;
}


/**************************************************************************************************
 * @fn          CanCcbAfProc
 *
 * @brief       Process the SB command and received buffer.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
 
uint8 CanCcbAfProc(UINT8 ucCcbIndex)
{
    switch(sbRxBuf[RPC_POS_CMD1])
    {
    case SAPP_CMD_DATA:
        CanCcbAfDataMsg(ucCcbIndex);
        break;
    }
    return FALSE;
}


/**************************************************************************************************
 * @fn          CanCcbAfProc
 *
 * @brief       Process the SB command and received buffer.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
 
uint8 CanCcbAfPeekDataMsg(UINT8 ucCcbIndex)
{
    APP_PACKET_COMM_STRU *pmg = (APP_PACKET_COMM_STRU *)&sbRxBuf[RPC_POS_DAT0]; 
    switch((pmg->ucMsgType & 0x7f))
    {
    case APP_PACKET_CLIENT_REPORT:
        if (!(pmg->ucMsgType & 0x80))
        {
            return CanCcbAfDataClientReportMsg(ucCcbIndex,(uint8_t *)pmg);
        }
        break;
    default: // yet to be implemented
        break;
    }

    return 0;
}


/**************************************************************************************************
 * @fn          CanCcbAfPeekProc
 *
 * @brief       peek and extract useful information from messages destinated to other hoster.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
 
uint8 CanCcbAfPeekProc(UINT8 ucCcbIndex)
{
    switch(sbRxBuf[RPC_POS_CMD1])
    {
    case SAPP_CMD_DATA:
        CanCcbAfPeekDataMsg(ucCcbIndex);
        break;
    }
    return FALSE;
}


void CanRcvFrame(UINT8 ucCcbIndex,UINT8 ucRcvBufIndex)
{
    UINT8 ucRet = TRUE;

    CanCcb[ucCcbIndex].usSrcAddr   = CanRcvBuff[ucRcvBufIndex].usSrcAddr;
    CanCcb[ucCcbIndex].ucDedicated = (ucRcvBufIndex == CAN_RCV_BUFFER_LOCAL_INDEX);
    CanCcb[ucCcbIndex].ucDataSrc   = 0;

    if (SHZNAPP_CanParse(CanRcvBuff[ucRcvBufIndex].head,CanRcvBuff[ucRcvBufIndex].len))
    {
        //MainAlarmWithDuration(1);

        CanHashAddr = CanAddress;

        sappItfType = Interface_CAN;

        switch(sbRxBuf[RPC_POS_CMD0])
        {
        case RPC_SYS_APP:
            if ((ucRcvBufIndex <= CAN_RCV_BUFFER_LOCAL_INDEX))
            {
                ucRet = SHZNAPP_SerialAppProc();
            }
            else
            {
                ucRet = FALSE;
            }
            break;
        case RPC_SYS_BOOT:
            if ((ucRcvBufIndex <= CAN_RCV_BUFFER_LOCAL_INDEX))
            {
                uint8_t ucCmd1 = sbRxBuf[RPC_POS_CMD1];

                ucRet = SHZNAPP_SerialBootProc();
                if (SBL_QUERY_ID_CMD == ucCmd1)
                {
                    if (INVALID_CAN_ADDRESS(CanHashAddr))
                    {
                        // calc hash address
                        CanHashAddr = CanCmdHashAdr();
                    }
                }
            }
            else
            {
                ucRet = FALSE;
            }
            break;
        case RPC_SYS_AF:
            if ((ucRcvBufIndex <= CAN_RCV_BUFFER_LOCAL_INDEX))
            {
                ucRet = CanCcbAfProc(ucCcbIndex);
            }
            else
            {
                ucRet = CanCcbAfPeekProc(ucCcbIndex);
            }
            break;
        case RPC_SYS_RCN: /* transparently directed to Zigbee */
            {
                zbLock(TRUE);
                zbCanItf(sbRxBuf);
                ucRet = FALSE;
            }
            break;
        default:
            ucRet = SHZNAPP_SerialUnknowProc();
            break;
        }

        if (ucRet)
        {
            (void)SHZNAPP_CanResp(CanRcvBuff[ucRcvBufIndex].usSrcAddr,CanHashAddr,0);  // Send the SB response setup in the sbBuf passed to sblProc().
        }
    }
}

int CanSndSappCmd(uint8_t ucCcbIdx,uint8_t ucCmd,uint8_t ucAddData,uint8_t *data, uint8_t len)
{
    switch(ucCcbIdx)
    {
    case CAN_LOCAL_CAN_INDEX:
         {
            sbTxBuf[RPC_POS_LEN]  = len; // len for data area (NOT INCLUDE CMD0&CMD1&LEN itself)
            sbTxBuf[RPC_POS_CMD0] = RPC_SYS_AF;
            sbTxBuf[RPC_POS_CMD1] = ucCmd;//SAPP_CMD_DATA;
            
            memcpy(&sbTxBuf[RPC_POS_DAT0],data,len);
            
            return SHZNAPP_CanResp(CanCcb[ucCcbIdx].usSrcAddr,CanAddress,ucAddData);
         }
    case CAN_ZIGBEE_INDEX:
         {

            int iOffset = RPC_POS_DAT0;
        
            sbTxBuf[RPC_POS_LEN]  = len + 3; // len for data area (NOT INCLUDE CMD0&CMD1&LEN itself)
            sbTxBuf[RPC_POS_CMD0] = RPC_SYS_APP;
            sbTxBuf[RPC_POS_CMD1] = ucCmd;//SAPP_CMD_DATA or SAPP_CMD_AIR_DATA;

            sbTxBuf[iOffset++]    = afAddr16Bit;
            sbTxBuf[iOffset++]    = 0; /* to coo */
            sbTxBuf[iOffset++]    = 0;

            /* protol type */
            if (ucCmd != SAPP_CMD_AIR_DATA)
            {
                sbTxBuf[iOffset++]    = ucCmd;
                sbTxBuf[RPC_POS_CMD1] = SAPP_CMD_DATA;
                sbTxBuf[RPC_POS_LEN] += 1;
            }
            
            memcpy(&sbTxBuf[iOffset],data,len);
            
            /* to zigbee */
            zb_SerialResp();

            return TRUE;
         }
    }
		
	return 0;
}

int CanSndSappCmd2(uint8_t ucCcbIdx,uint16 usDstCanAdr,uint16 usSrcCanAdr,uint8_t ucCmd,uint8_t *data, uint8_t len)
{
    switch(ucCcbIdx)
    {
    case CAN_LOCAL_CAN_INDEX:
        {

            sbTxBuf[RPC_POS_LEN]  = len; // len for data area (NOT INCLUDE CMD0&CMD1&LEN itself)
            sbTxBuf[RPC_POS_CMD0] = RPC_SYS_AF;
            sbTxBuf[RPC_POS_CMD1] = ucCmd;//SAPP_CMD_DATA;
            
            memcpy(&sbTxBuf[RPC_POS_DAT0],data,len);
        
            return SHZNAPP_CanResp(usDstCanAdr,usSrcCanAdr,0);
        }
    case CAN_ZIGBEE_INDEX:
        {
   
           int iOffset = RPC_POS_DAT0;
       
           sbTxBuf[RPC_POS_LEN]  = len + 3; // len for data area (NOT INCLUDE CMD0&CMD1&LEN itself)
           sbTxBuf[RPC_POS_CMD0] = RPC_SYS_APP;
           sbTxBuf[RPC_POS_CMD1] = ucCmd;//SAPP_CMD_DATA;
   
           sbTxBuf[iOffset++]    = afAddr16Bit;
           sbTxBuf[iOffset++]    = 0; /* to coo */
           sbTxBuf[iOffset++]    = 0;

           /* protol type */
           if (ucCmd != SAPP_CMD_AIR_DATA)
           {
               sbTxBuf[iOffset++]    = ucCmd;
               sbTxBuf[RPC_POS_CMD1] = SAPP_CMD_DATA;
               sbTxBuf[RPC_POS_LEN] += 1;
           }
   
           memcpy(&sbTxBuf[iOffset],data,len);
           
           /* to zigbee */
           zb_SerialResp();
        }
         break;        
    }
		
    return 0;
}

int CanSndSappRawCmd2(uint16 usDstCanAdr,uint16 usSrcCanAdr,uint8_t ucSys,uint8_t ucCmd,uint8_t *data, uint8_t len)
{
    sbTxBuf[RPC_POS_LEN]  = len; // len for data area (NOT INCLUDE CMD0&CMD1&LEN itself)
    sbTxBuf[RPC_POS_CMD0] = ucSys;
    sbTxBuf[RPC_POS_CMD1] = ucCmd;//SAPP_CMD_DATA;
    
    memcpy(&sbTxBuf[RPC_POS_DAT0],data,len);

    return SHZNAPP_CanResp(usDstCanAdr,usSrcCanAdr,0);
}


void CanCmdRegisterSndEmptyCallBack(CAN_Snd_Empty_CallBack cb)
{
    gCanSndEmptyCB = cb;
}

void CanCmd_msg_Handler(Message *Msg)
{
    CANCMD_MSG *dmsg = (CANCMD_MSG *)Msg;

    if (dmsg->para)
    {
        ((cancmd_msg_cb)dmsg->para)(dmsg->para1);
    }
}

void CanCcbRegister_msg_handler(void *para)
{
    // task
	char buf[32];
	int iIdx = 0;
    UINT8 ucIndex = (UINT8)(UINT32)para;
    APP_PACKET_ONLINE_NOTI_IND_STRU *ind = (APP_PACKET_ONLINE_NOTI_IND_STRU *)buf;
    
    ind->hdr.ucTransId   = APP_DEV_TYPE_HAND_SET;
    ind->hdr.ucDevType   = APP_DEV_TYPE_HAND_SET;
    ind->hdr.ucMsgType   = APP_PAKCET_COMM_ONLINE_NOTI;
    ind->acInfo[iIdx++]  = (uint8_t)CanAddress; /* reserved */
    memcpy(&ind->acInfo[iIdx],gCfg.cfg1.serial,CONFIG_SERIAL_NAME_LEN);
    iIdx                += CONFIG_SERIAL_NAME_LEN;

    ind->hdr.ucLen       = iIdx;
    
    // set to main controller's address
    CanCcb[ucIndex].usSrcAddr = APP_DEV_TYPE_MAIN_CTRL; 

    // push to queue
    if (CanSndSappCmd(ucIndex ,SAPP_CMD_DATA,0,(uint8_t *)ind,APP_PROTOL_HEADER_LEN + ind->hdr.ucLen))
	{
		// report device info to hosts
		AddTimer(TIMER_CCB_BEGIN + ucIndex,OS_TMR_OPT_ONE_SHOT,CAN_LOGIC_DEVICE_REGISETER_PERIOD/1000*OS_TMR_CFG_TICKS_PER_SEC,0XFFFF);
		
		CanCcb[ucIndex].ucMachineState = MACHINE_STATE_MAIN_WAIT_REGISTER_RSP;
		
	}
}


uint8_t CanCcbSndHandleOperationMsg(UINT8 ucIndex,uint8_t *pucMsg,uint8_t ucLength)
{
    int iTimerLen = CAN_WAIT_RSP_TIEMR_LENGTH;
    
    APP_PACKET_HO_STRU *pOpsMsg = (APP_PACKET_HO_STRU *)pucMsg;
    
    // report device info to host
    if (MACHINE_STATE_IDLE != CanCcb[ucIndex].ucMachineState)
    {
        return 0XFF;
    }

    if (APP_PACKET_HO_QTW == pOpsMsg->ucOpsType)
    {
        iTimerLen = CAN_WAIT_TW_RSP_TIEMR_LENGTH;
    }

    // set to main controller's address
    CanCcb[ucIndex].usSrcAddr = APP_DEV_TYPE_MAIN_CTRL; 

    // push to queue
    if (CanSndSappCmd(ucIndex ,SAPP_CMD_DATA,1,pucMsg,ucLength))
	{
	    if (!(pOpsMsg->hdr.ucMsgType & 0x80))
        {   
    		AddTimer(TIMER_CCB_BEGIN + ucIndex,OS_TMR_OPT_ONE_SHOT,(iTimerLen/1000*OS_TMR_CFG_TICKS_PER_SEC),pOpsMsg->ucOpsType);
    		
    		CanCcb[ucIndex].ucMachineState = MACHINE_STATE_MAIN_WAIT_OPRATION_RSP;
        }

        //VOS_LOG(VOS_LOG_ERROR,"send Ops  %d&%d\r\n",ucIndex,pOpsMsg->ucOpsType);

        return 0;
	}
	else
	{
		VOS_LOG(VOS_LOG_DEBUG,"CanSndSappCmd failed\r\n");
	}

    return 0;
}

uint8_t CanCcbSndZigbeeIndMsg(void)
{
    uint8_t buf[16];
    
    APP_PACKET_ZIGBEE_IND_STRU *pLoad = (APP_PACKET_ZIGBEE_IND_STRU *)buf;

    if (INVALID_CAN_ADDRESS(CanAddress))
    {
        return 0XFF;
    }
    
    pLoad->hdr.ucLen     = 2;
    pLoad->hdr.ucTransId = 0;
    pLoad->hdr.ucDevType = APP_DEV_TYPE_HAND_SET;
    pLoad->hdr.ucMsgType = APP_PACKET_COMM_ZIGBEE_IND;

    pLoad->usAddress     = CanAddress;

    // set to main controller's address
    CanCcb[CAN_ZIGBEE_INDEX].usSrcAddr = APP_DEV_TYPE_MAIN_CTRL; 

    // push to queue
    return CanSndSappCmd(CAN_ZIGBEE_INDEX ,SAPP_CMD_DATA,0,buf,pLoad->hdr.ucLen + APP_PROTOL_HEADER_LEN);
}

void CanCcb_Register(void *para)
{
     CanCmd_report(CanCcbRegister_msg_handler,para);
}


void CanCcbDeviceCommDeviceInfoRegister(UINT8 ucIndex)
{

    if (MACHINE_STATE_IDLE != CanCcb[ucIndex].ucMachineState)
    {
        // Ongoing procedure is waiting response from network controller
        // discard the message
        return;
    }

    if (CanCcb[ucIndex].bit1Registered)
    {
        return ; 
    }


    if  (CAN_LOCAL_CAN_INDEX == ucIndex
        && (CanCcb[ucIndex].ucRegTryNum >= 3)
        && CanCcb[CAN_ZIGBEE_INDEX].bit1Registered)
    {
        return;
    }

    if (CanCcb[ucIndex].ucRegTryNum < 0XFF)
    {
        CanCcb[ucIndex].ucRegTryNum++;
    }

    sys_timeout(CAN_CMD_RSP_DELAY(20),SYS_TIMER_ONE_SHOT,CAN_CMD_RSP_DELAY(20),CanCcb_Register,(void *)ucIndex,&CanCcb[ucIndex].to4DelayExcu);

}



/*********************************************************************
 * Function:        void CanCcbSystemStartReportRegister(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call Once when system is up to notify net controller of local setting 
 *
 * Note:            None.
 ********************************************************************/
void CanCcbSystemStartReportRegister(uint8_t ucTrxIndex)
{
    if (INVALID_CAN_ADDRESS(CanAddress)) 
    {
        /* Make requeset to controller */    
        return;
    }
    
    CanCcbDeviceCommDeviceInfoRegister(ucTrxIndex);

}

/* Callback */

void CanCcbSecondTask(void)
{
    CanCcbSystemStartReportRegister(CAN_LOCAL_CAN_INDEX);
    
    CanBusyCheck();

}


