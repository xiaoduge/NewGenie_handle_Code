#include    <ucos_ii.h>

#include    <string.h>

#include "common.h"

#include "osal_snv.h"

#include "memory.h"
#include "task.h"
#include "timer.h"

#include "stm32_eval.h"

#include "app_cfg.h"

#include "serial_driver.h"

#include "serial.h"

#include "Config.h"

#include "UartCmd.h"


SERIAL_STRU Serial[SERIAL_MAX_NUMBER];

//USART_TypeDef  * const Serial_UsartDef[SERIAL_MAX_NUMBER] = {EVAL_COM1,EVAL_COM2,EVAL_COM3,EVAL_COM4,EVAL_COM5};
//const int Serial_Irq[SERIAL_MAX_NUMBER] = {EVAL_COM1_IRQn,EVAL_COM2_IRQn,EVAL_COM3_IRQn,EVAL_COM4_IRQn,EVAL_COM5_IRQn};
//const COM_TypeDef Serial_Com[SERIAL_MAX_NUMBER] = {COM1,COM2,COM3,COM4,COM5};


// SERIAL_CONFIG_STRU SerialConfig[SERIAL_MAX_NUMBER];

const uint16_t Serial_DataBits[BAUD_DATA_BITS_NUM]  = {USART_WordLength_8b,USART_WordLength_9b};
const uint16_t Serial_StopBits[BAUD_STOP_BITS_NUM]  = {USART_StopBits_1,USART_StopBits_1_5,USART_StopBits_2};
const uint16_t Serial_Parity[BAUD_PARITY_NUM]       = {USART_Parity_No,USART_Parity_Even,USART_Parity_Odd};
//const uint8_t Serial_PortCtrl[SERIAL_MAX_NUMBER]    = {0,STM32F103_GPA(15)};
//const UINT8 Serial_PortType[SERIAL_MAX_NUMBER]      = {RS232,RS485};

static int  sSerialInited[SERIAL_MAX_NUMBER] = {0,0,0,
#ifdef STM32F10X_HD     
                                                0,0,
#endif
} ;

#define MB_SERIAL ((void *)IPC_DRIVER)

void SerialInitPort(UINT8 ucPort)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    USART_InitTypeDef USART_InitStructure;

    if (!Serial[ucPort].sccb)
    {
        return ; // serial port not enabled
    }

    Serial[ucPort].sccb(ucPort);

    if (!Serial[ucPort].UsartDef)
    {
        return ; // serial port not enabled
    }
    
    /* Enable the USARTx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = Serial[ucPort].iIrq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate   = Serial[ucPort].SerialConfig.BaundRate;
    USART_InitStructure.USART_WordLength = Serial_DataBits[Serial[ucPort].SerialConfig.ucDataBits];
    USART_InitStructure.USART_StopBits   = Serial_StopBits[Serial[ucPort].SerialConfig.ucStopBits];
    USART_InitStructure.USART_Parity     = Serial_Parity[Serial[ucPort].SerialConfig.ucParity];
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    STM_EVAL_COMInit((COM_TypeDef)Serial[ucPort].iComIdx, &USART_InitStructure); // usart2 for RS485

    USART_ITConfig(Serial[ucPort].UsartDef, USART_IT_RXNE, ENABLE);

    if (!sSerialInited[ucPort])
    {
        UINT8 ucTmp = Serial[ucPort].ucDriverType;
        sSerialInited[ucPort] = 1;
        Serial[ucPort].ucDriverType = MSG_DRIVER;
        Serial[ucPort].mb = NULL;
        Serial_SetDriverType(ucPort,ucTmp);
    }
    
    Serial[ucPort].usRcvIdleTime = Serial[ucPort].SerialConfig.ucIdleCheckThd; // 100ms

    if (RS485 == Serial[ucPort].ucPortType)
    {
        stm32_gpio_cfgpin(Serial[ucPort].ucPortCtrl,MAKE_PIN_CFG(GPIO_Speed_50MHz,GPIO_Mode_Out_PP)); // YLF: FOR RS485 CONTROL
    
        stm32_gpio_set_value(Serial[ucPort].ucPortCtrl,0); // ylf:  output low
    }

}

void SerialEnableTx(UINT8 ucPort,UINT8 ucEnable)
{
    if (RS485 != Serial[ucPort].ucPortType)
    {
        return;
    }
    if (ucEnable)
    {
        stm32_gpio_set_value(Serial[ucPort].ucPortCtrl,1); // ylf:  output high
    }
    else
    {
	    stm32_gpio_set_value(Serial[ucPort].ucPortCtrl,0); // ylf:  output low
    }
}


void SerialInit(void)
{
    int ucLoop = 0;
    for (ucLoop = 0; ucLoop < SERIAL_MAX_NUMBER;ucLoop++)
    {
        SerialInitPort(ucLoop);
    }

}

UINT8  Serial_FillSndBuf(UINT8 ucPort ,UINT8 *pData,UINT16 usLength)
{
    UINT16 usSpace ;

    if (ucPort >= SERIAL_MAX_NUMBER)
    {
        return FALSE;
    }

    usSpace = Serial[ucPort].usSndRear == Serial[ucPort].usSndFront ? SERIAL_MAX_SEND_BUFF_LENGTH : (SERIAL_MAX_SEND_BUFF_LENGTH + Serial[ucPort].usSndRear - Serial[ucPort].usSndFront) % SERIAL_MAX_SEND_BUFF_LENGTH;

    if (usSpace < usLength)
    {
        return FALSE;
    }

    SerialEnableTx(ucPort,TRUE);

    if (Serial[ucPort].usSndFront >= Serial[ucPort].usSndRear)
    {
       if (usLength < SERIAL_MAX_SEND_BUFF_LENGTH - Serial[ucPort].usSndFront) 
       {
            memcpy(&Serial[ucPort].SndBuff[Serial[ucPort].usSndFront],pData,usLength);
            Serial[ucPort].usSndFront += usLength;
       }
       else
       {
           memcpy(&Serial[ucPort].SndBuff[Serial[ucPort].usSndFront],pData,SERIAL_MAX_SEND_BUFF_LENGTH - Serial[ucPort].usSndFront);
           pData += (SERIAL_MAX_SEND_BUFF_LENGTH - Serial[ucPort].usSndFront);
           usLength -= (SERIAL_MAX_SEND_BUFF_LENGTH - Serial[ucPort].usSndFront);
           memcpy(&Serial[ucPort].SndBuff[0],pData,usLength);
           Serial[ucPort].usSndFront = usLength;
       }
    }
    else
    {
        memcpy(&Serial[ucPort].SndBuff[Serial[ucPort].usSndFront],pData,usLength);
        Serial[ucPort].usSndFront += usLength;
    }

    
    USART_ITConfig(Serial[ucPort].UsartDef, USART_IT_TXE, ENABLE);

    return TRUE;
}

UINT8  Serial_FillRcvBuf(UINT8 ucPort ,UINT8 *pData,UINT16 usLength)
{
    UINT16 usSpace = Serial[ucPort].usRcvRear == Serial[ucPort].usRcvFront ? SERIAL_MAX_RECV_BUFF_LENGTH : (SERIAL_MAX_RECV_BUFF_LENGTH + Serial[ucPort].usRcvRear - Serial[ucPort].usRcvFront) % SERIAL_MAX_RECV_BUFF_LENGTH;

    if (usSpace < usLength)
    {
        return FALSE;
    }

    if (Serial[ucPort].usRcvFront >= Serial[ucPort].usRcvRear)
    {
       if (usLength < SERIAL_MAX_RECV_BUFF_LENGTH - Serial[ucPort].usRcvFront) 
       {
            memcpy(&Serial[ucPort].RcvBuff[Serial[ucPort].usRcvFront],pData,usLength);
            Serial[ucPort].usRcvFront += usLength;
       }
       else
       {
           memcpy(&Serial[ucPort].RcvBuff[Serial[ucPort].usRcvFront],pData,SERIAL_MAX_RECV_BUFF_LENGTH - Serial[ucPort].usRcvFront);
           pData += (SERIAL_MAX_RECV_BUFF_LENGTH - Serial[ucPort].usRcvFront);
           usLength -= (SERIAL_MAX_RECV_BUFF_LENGTH - Serial[ucPort].usRcvFront);
           memcpy(&Serial[ucPort].RcvBuff[0],pData,usLength);
           Serial[ucPort].usRcvFront = usLength;
       }
    }
    else
    {
        memcpy(&Serial[ucPort].RcvBuff[Serial[ucPort].usRcvFront],pData,usLength);
        Serial[ucPort].usRcvFront += usLength;
    }

    Serial_ResetRcvIdleTime(ucPort);

    return TRUE;

}

void Serial_SetRcvIdleTime(UINT8 ucPort,UINT16 usRcvIdleTime)
{
    Serial[ucPort].usRcvIdleTime = usRcvIdleTime;
    Serial[ucPort].usRcvIdleCnt = usRcvIdleTime;
}

int Serial_MoveData(UINT8 ucPort,char *data,INT8U length)
{

    UINT16 usDist = (SERIAL_MAX_RECV_BUFF_LENGTH - Serial[ucPort].usRcvRear + Serial[ucPort].usRcvFront) % SERIAL_MAX_RECV_BUFF_LENGTH;

    if (length >= usDist)   
    {
        length = usDist;
    }

    if (length > 0)
    {
    
        UINT16 usLeft;
#if OS_CRITICAL_METHOD == 3                                /* Allocate storage for CPU status register */
        OS_CPU_SR     cpu_sr = 0;
#endif
    
        OS_ENTER_CRITICAL();
    
        if (Serial[ucPort].usRcvRear < Serial[ucPort].usRcvFront)
        {
            memcpy(data,&Serial[ucPort].RcvBuff[Serial[ucPort].usRcvRear],length);
        }
        else
        {
            if (SERIAL_MAX_RECV_BUFF_LENGTH - Serial[ucPort].usRcvRear <= length)
            {
                memcpy(data,&Serial[ucPort].RcvBuff[Serial[ucPort].usRcvRear],SERIAL_MAX_RECV_BUFF_LENGTH - Serial[ucPort].usRcvRear);
    
                usLeft = length - (SERIAL_MAX_RECV_BUFF_LENGTH - Serial[ucPort].usRcvRear);
            }
            else
            {
                memcpy(data,&Serial[ucPort].RcvBuff[Serial[ucPort].usRcvRear],length);
                
                usLeft = 0;
            }
            
            if (usLeft > 0 )
            {
                memcpy(&data[SERIAL_MAX_RECV_BUFF_LENGTH - Serial[ucPort].usRcvRear],&Serial[ucPort].RcvBuff[0],usLeft);
            }
        }
    
        Serial[ucPort].usRcvRear = (Serial[ucPort].usRcvRear + length) % SERIAL_MAX_RECV_BUFF_LENGTH;
    
        if (Serial[ucPort].usRcvRear == Serial[ucPort].usRcvFront)
        {
            Serial[ucPort].usRcvRear = Serial[ucPort].usRcvFront = 0;
        }
        OS_EXIT_CRITICAL();
    }

    return length;
}

void Serial_CheckRcvIdleTime(UINT8 ucPort)
{

    if (Serial[ucPort].usRcvRear == Serial[ucPort].usRcvFront)
    {
        // empty ,do nothing
        return ; 
    }
    
    Serial[ucPort].usRcvIdleCnt++;

    if (Serial[ucPort].usRcvIdleCnt >= Serial[ucPort].usRcvIdleTime)
    {
        Message *Msg;
        UINT16 usDist = (SERIAL_MAX_RECV_BUFF_LENGTH - Serial[ucPort].usRcvRear + Serial[ucPort].usRcvFront) % SERIAL_MAX_RECV_BUFF_LENGTH;
        if (MSG_DRIVER == Serial[ucPort].ucDriverType)
        {
            // copy
            Msg = MessageAlloc(PID_RS485,usDist);
            
            if (Msg)
            {   
                Serial_MoveData(ucPort,Msg->data,usDist);                
                Msg->msgHead.nMsgType = ucPort;
                MessageSend(Msg);
            }
        }  
        else
        {
            OSMboxPost(Serial[ucPort].mb,MB_SERIAL);
        }
    }

}

void Serial_ResetRcvIdleTime(UINT8 ucPort)
{
    Serial[ucPort].usRcvIdleCnt = 0;

}

void Serial_RetriveConfig(UINT8 ucPort)
{
    //memcpy(&SerialConfig[ucPort],&pParameter[PARAMETER_SERIAL_BEGIN_POS+PARAMETER_SERIAL_SIZE*ucPort],PARAMETER_SERIAL_SIZE);

    osalSnvId_t nvid = calcSerialNvID(ucPort);

//    uint8 ucValidCfg = TRUE;

    
    if ( osal_snv_read( nvid, sizeof ( SERIAL_CONFIG_STRU ), &Serial[ucPort].SerialConfig ) != ERROR_SUCCESS )
    {
        //ucValidCfg = FALSE;
        
        memset(&Serial[ucPort].SerialConfig,0xff,sizeof ( SERIAL_CONFIG_STRU ));
    }
        

    if (Serial[ucPort].SerialConfig.BaundRate > 115200)
    {
        Serial[ucPort].SerialConfig.BaundRate = 9600;
    }

    if (Serial[ucPort].SerialConfig.ucDataBits >= BAUD_DATA_BITS_NUM)
    {
        Serial[ucPort].SerialConfig.ucDataBits = BAUD_DATA_8BITS;
    }
    
    if (Serial[ucPort].SerialConfig.ucStopBits >= BAUD_STOP_BITS_NUM)
    {
        Serial[ucPort].SerialConfig.ucStopBits = BAUD_STOP_1BITS;
    }
    
    if (Serial[ucPort].SerialConfig.ucParity >= BAUD_PARITY_NUM)
    {
        Serial[ucPort].SerialConfig.ucParity = BAUD_PARITY_NO;
    }

    if (Serial[ucPort].SerialConfig.ucIdleCheckThd > 250 )
    {
        Serial[ucPort].SerialConfig.ucIdleCheckThd = 2; // 20ms  is enough for modbus protocol
    }
    else if (Serial[ucPort].SerialConfig.ucIdleCheckThd < 2)
    {
        Serial[ucPort].SerialConfig.ucIdleCheckThd = 2; // 10~20ms is enought for modbus protocol
    }
    
}

/**
  * @brief  Configures Serial Port Driver Type, MUST be called after serial port initialized.
  * @param  ucPort: Specifies the COM port to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg SERIAL_PORT_ENUM
  * @param  ucDrvType: specifiy the driver type
  *   This parameter can be one of following parameters:    
  *     @arg SERIAL_DRIVER_TYPE_ENUM
  * @retval None
  */
void Serial_SetDriverType(UINT8 ucPort,UINT8 ucDrvType)
{
    if (ucDrvType == Serial[ucPort].ucDriverType)
    {
        return ;
    }

    if (MSG_DRIVER == ucDrvType)
    {
        if (Serial[ucPort].mb) 
        {
            INT8U err;
            OSMboxPendAbort(Serial[ucPort].mb,OS_PEND_OPT_BROADCAST,&err);
        }
        Serial[ucPort].ucDriverType = ucDrvType;
    }
    else
    {
        if (!Serial[ucPort].mb) 
        {
            Serial[ucPort].mb = OSMboxCreate(0);  
            
            if (Serial[ucPort].mb)
            {
                Serial[ucPort].ucDriverType = ucDrvType;
            }
        }        
    }

}

/**
  * @brief  get serial driver mode.
  * @param  ucPort: Specifies the COM port to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg SERIAL_PORT_ENUM
  * @retval mode
  */
int Serial_GetDrvMode(UINT8 ucPort)
{
    return Serial[ucPort].ucDriverType;
}



int Serial_Rcv_Empty(INT8U ucPort)
{
    return (Serial[ucPort].usRcvRear == Serial[ucPort].usRcvFront);
}

void Serial_ReInit(UINT8 ucPort)
{
    USART_DeInit(Serial[ucPort].UsartDef);

    SerialInitPort(ucPort);
}

/**
  * @brief  get serial driver mode.
  * @param  ucPort: Specifies the COM port to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg SERIAL_PORT_ENUM
  * @retval mode
  */
int Serial_GetSndBufferSpace(UINT8 ucPort)
{
    return (SERIAL_MAX_SEND_BUFF_LENGTH + Serial[ucPort].usSndRear - Serial[ucPort].usSndFront) % SERIAL_MAX_SEND_BUFF_LENGTH;
}

#if (SERIAL_SUPPORT > 0)
UINT8 PidSerialProcess(Message *pMsg)
{
    INT8U ucPort = pMsg->msgHead.nMsgType;

    {
        // transfer control to CAN interface
        if (ucPort < SERIAL_MAX_NUMBER)
        {
            if (Serial[ucPort].mcb)
            {
                Serial[ucPort].mcb(pMsg);
            }
        }
    }

	return 0;
}
#endif
