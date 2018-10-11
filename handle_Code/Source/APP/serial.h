#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "DtypeStm32.h"
#include "config.h"

enum SERIAL_PORT_TYPE_ENUM
{
    RS485 = 0,
    RS232 ,
};

enum SERIAL_DRIVER_TYPE_ENUM
{
    MSG_DRIVER = 0,
    IPC_DRIVER ,
};


enum SERIAL_PORT_ENUM // refer COM_TypeDef
{
    SERIAL_PORT0 = 0,
    SERIAL_PORT1 ,
    SERIAL_PORT2 ,
#ifdef STM32F10X_HD     
    SERIAL_PORT3 ,
    SERIAL_PORT4 ,
#endif    
    SERIAL_PORT_NUM,
};


#define SERIAL_MAX_NUMBER (SERIAL_PORT_NUM)

#define SERIAL_MAX_SEND_BUFF_LENGTH 128
#define SERIAL_MAX_RECV_BUFF_LENGTH 256

typedef void (*serial_config_cb)(UINT8 ucPort);
typedef void (*serial_msg_cb)(Message *msg);

typedef struct
{
    
    UINT8  SndBuff[SERIAL_MAX_SEND_BUFF_LENGTH];
    UINT8  RcvBuff[SERIAL_MAX_RECV_BUFF_LENGTH];

    UINT16 usSndFront,usSndRear;
    UINT16 usRcvFront,usRcvRear;

    UINT16 usRcvIdleTime;

    UINT16 usRcvIdleCnt;


    UINT8 ucDriverType; // SERIAL_DRIVER_TYPE_ENUM

    // following must put below ucDriverType
    USART_TypeDef  *UsartDef;
    int             iIrq;
    int             iComIdx;
    UINT8 ucPortType;   // SERIAL_PORT_TYPE_ENUM
    SERIAL_CONFIG_STRU SerialConfig;
    UINT8 ucPortCtrl;   // ctrl pin for rs485    
    serial_config_cb    sccb; // for serial config call back
    serial_msg_cb       mcb;

    OS_EVENT  *mb;   // VALID When Driver by IPC
}SERIAL_STRU;


extern SERIAL_STRU Serial[SERIAL_MAX_NUMBER];

#define SERIAL_EMPTY(Front,Rear)((Front)==(Rear))

#define SERIAL_FULL(Front,Rear,limit)((Front+1)%limit==(Rear))


void SerialInit(void);

UINT8  Serial_FillRcvBuf(UINT8 ucPort,UINT8 *pData,UINT16 usLength);

UINT8  Serial_FillSndBuf(UINT8 ucPort,UINT8 *pData,UINT16 usLength);
UINT8 PidSerialProcess(Message *pMsg);
void Serial_SetRcvIdleTime(UINT8 ucPort,UINT16 usRcvIdleTime);
void Serial_CheckRcvIdleTime(UINT8 ucPort);
void Serial_ResetRcvIdleTime(UINT8 ucPort);
void Serial_RetriveConfig(UINT8 ucPort);
void Serial_ReInit(UINT8 ucPort);
void Serial_SetDriverType(UINT8 ucPort,UINT8 ucDrvType);
void SerialInitPort(UINT8 ucPort);
void Serial_SndTest(UINT8 ucPort ,UINT8 *pData,UINT16 usLength);
void ConfigRs485ReadCfg(void);
void SerialEnableTx(UINT8 ucPort,UINT8 ucEnable);
int Serial_Read(INT8U ucPort,char *data,INT8U length,INT16U tmout);
int Serial_GetDrvMode(UINT8 ucPort);
int Serial_GetSndBufferSpace(UINT8 ucPort);

#endif
