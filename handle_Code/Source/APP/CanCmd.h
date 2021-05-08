#ifndef _CAN_CMD_H_
#define _CAN_CMD_H_


#include "stm32f10x_can.h"

#include "DtypeStm32.h"

#include "msg.h"

#include "sys_time.h"

#ifndef MAX_CAN_CCB_NUMBER
#define MAX_CAN_CCB_NUMBER  3
#endif

#define CAN_MAX_BUSY_SECOND 180

//#define SIMULATOR

/********************
   buffer for CAN segment and reassemble
********************/
typedef struct  {

    UINT16      len;     // total CAN network layer message len
    UINT16      data_len; // how many data left to  be received

    UINT8       *head;   // memory buffer head pointer
    UINT8       *dat;   // current data pos

    UINT8       ucSubCanAdr; // rcv buffer index
    UINT16      usSrcAddr;
}CAN_Rcv_buff;

typedef struct  {

    UINT16      len;     // total CAN network layer message len
    UINT16      data_len; // how many data left to have been sended 

    UINT8       *head;   // memory buffer head pointer
    UINT8       *dat;   // current data pos

    UINT8       ucPort;       
        

    UINT8       aucCmd[2];

    UINT8       ucCcbIndex;

    UINT8       ucCanAdrType;

    UINT16      usSrcCanAddress;

    UINT16      usDstCanAddress;
    
}CAN_Snd_buff;

typedef enum
{
   MACHINE_STATE_IDLE = 0,
   MACHINE_STATE_MAIN_WAIT_REGISTER_RSP ,
   MACHINE_STATE_MAIN_WAIT_OPRATION_RSP,
   MACHINE_STATE_MAIN_WAIT_TW_RSP,
   
   MACHINE_STATE_BUTT
}MACHINE_STATE_ENUM;

/********************
   CAN Call control block
********************/
typedef struct
{
    uint32_t bit1OnlineState : 1;
    uint32_t bit1HostReset   : 1; // for controller only
    uint32_t bit1Registered  : 1;
    uint32_t bit3HeartBeat   : 3;

    uint16_t usHeartBeatPeriod;

    uint8_t ucCcbIdx;
    uint8_t ucState;

    // for machine state 
    uint8_t ucMachineState; // refer MACHINE_STATE_ENUM
    uint8_t ucStateTimerId;

    // for temperory information
    uint16_t usSrcAddr;
    uint8_t  ucDedicated;
    uint8_t ucDataSrc; // 0 for can, 1 for zigbee

    uint8_t ucInactCnt;
    uint8_t ucRegTryNum;

    sys_timeo to4HB;

    sys_timeo to4Callback;
    
    sys_timeo to4DelayExcu;

    
}CAN_CCB;


typedef void (*CAN_Snd_Empty_CallBack)(void);

typedef enum
{
   CAN_BROADCAST_INDEX = 0x0,
   CAN_LOCAL_CAN_INDEX ,
   CAN_ZIGBEE_INDEX,
   CAN_ADR_BUTT
}CAN_ADR_TYPE_ENUM;

typedef enum
{
   CAN_RCV_BUFFER_COMMON_INDEX = 0x0,
   CAN_RCV_BUFFER_LOCAL_INDEX ,
   CAN_RCV_BUFFER_BUTT
}CAN_RCV_BUFFER_ENUM;

#define MAX_CAN_OUTPUT_BUFFER 8

#define CAN_MAX_RCV_BUFFER MAX_CAN_OUTPUT_BUFFER

extern UINT16 CanAddress ;
extern UINT16 CanHashAddr;

extern CAN_CCB  CanCcb[MAX_CAN_CCB_NUMBER];

extern const char *rcdFileName[MAX_CAN_CCB_NUMBER];

#define  MAX_CCB_MASK_LENGTH ((MAX_CAN_CCB_NUMBER + 31)/32)

#define CANCMD_SET_CAN_MASK(array,idx) (array[idx/32] |= (1 << (idx%32)))
#define CANCMD_RESET_CAN_MASK(array,idx) (array[idx/32] &= ~(1 << (idx%32)))
#define CANCMD_GET_CAN_MASK(array,idx) (!!(array[idx/32] & (1 << (idx%32))))


#if (APP_TYPE == APP_TYPE_CTRL)
extern uint32_t gaulRegisteredCcbMask[MAX_CCB_MASK_LENGTH];
#endif

void CanCmdInit(void);
UINT8 PidCanProcess(Message *pMsg);
void CanBusyCheck(void);
UINT8 CanCmdIapSetAddr(UINT8 *data);
void CanCmdInitizeCAN(void);
void CanRcvFrame(UINT8 ucCcbIndex,UINT8 ucRcvBufIndex);
int CanSndSappCmd(UINT8 ucCcbIndex,uint8_t ucCmd,uint8_t ucAddData,uint8_t *data, uint8_t len);
int CanSndSappCmd2(UINT8 ucCcbIndex,uint16_t usDstCanAdr,uint16_t usSrcCanAdr,uint8_t ucCmd,uint8_t *data, uint8_t len);
int CanSndSappRawCmd2(uint16_t usDstCanAdr,uint16_t usSrcCanAdr,uint8_t ucSys,uint8_t ucCmd,uint8_t *data, uint8_t len);

void CanCmdRegisterSndEmptyCallBack(CAN_Snd_Empty_CallBack cb);
void CanCcbSecondTask(void);
void CanCcbTimerProc(uint8_t ucTimeId,uint16_t AddData);
uint8_t GasMeas_Itf(uint8_t ucCcbIdx,uint16_t usAdcValue);
int GasMeas_GetDec(uint8_t ucDec);
uint8_t ConfigGetGasUnit(uint8_t ucGasType);
void Disp_CanItfNotify(uint8_t ucTrxIndex,int code);
char *getRcdFileName(int iChl,int iEvent);
uint8_t CanCcbAfDataClientSndOpsIndMsg(UINT8 ucCcbIndex,UINT8 ucOps);
void CanCmd_msg_Handler(Message *Msg);
void CanCcbTimeout(UINT8 ucIndex,UINT16 usPara);
uint8_t CanCcbSndHandleOperationMsg(UINT8 ucIndex,uint8_t *pucMsg,uint8_t ucLength);
void CanCmdSetAdr(uint16_t usAddress);
UINT16 CanCmdHashAdr(void);
void CanCcbResetRegisterFlag(UINT8 ucCcbIndex);
uint8_t CanCcbAfProc(UINT8 ucCcbIndex,UINT8 *sbRxBuf);
void CanCcbSystemStartReportRegister(uint8_t ucTrxIndex);
uint8_t CanCcbSndZigbeeIndMsg(void);
uint8_t Trx2CanCcbIndex(uint8_t ucIndex);
uint8_t CanCcbIndex2Trx(uint8_t ucCcbIndex);

#endif

