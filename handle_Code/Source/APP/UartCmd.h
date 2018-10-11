#ifndef _UART_CMD_H_
#define _UART_CMD_H_

#include "memory.h"
#include "msg.h"
#include "timer.h"

#include "hal_types.h"

/* UART Events */
#define HAL_UART_RX_FULL         0x01
#define HAL_UART_RX_ABOUT_FULL   0x02
#define HAL_UART_RX_TIMEOUT      0x04
#define HAL_UART_TX_FULL         0x08
#define HAL_UART_TX_EMPTY        0x10


#define MAX_RCV_BUFF_LEN 256
#define MAX_SND_BUFF_LEN 256

typedef void (*halUARTCBack_t) (uint8 port, uint8 event);

typedef struct
{
    UINT8 UART_Rcvbuff[MAX_RCV_BUFF_LEN];
    UINT8 UART_Sndbuff[MAX_SND_BUFF_LEN];
    
    UINT8 rcvfront,rcvrear;

    halUARTCBack_t cb;

}UART_CMD_STRU;

void UartCmdInit(void);

void UartCmdPutData(UINT8 ucData);

//UINT8 PidUartCmdProcess(Message *pMsg);

void UartCmdSendMsg(UINT8 *pData,UINT8 ucLength);

uint16 HalUARTWrite(uint8 port, uint8 *pBuffer, uint16 length);

uint16 HalUARTRead(uint8 port, uint8 *pBuffer, uint16 length);

void UartCmdSerialWrite(uint8 ucCmd,uint8 *pBuffer, uint16 length);

void UartCmdRs485Report(UINT8 ucPort ,UINT8 *pData,UINT8 ucLength);
void UartCmdPrintf(int level,const char* fmt,...);
void UartCmdSetLogLevel(int iLevel);

#ifdef SERIAL_DEBUG
#define DEBUG_LOG(fmt, args...)   UartCmdPrintf(fmt,##  args)
#else
#define DEBUG_LOG(fmt, args...)   
#endif

#endif
