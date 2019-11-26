#include    <ucos_ii.h>

#include    <cpu.h>

#include    <string.h>

#include    <stdio.h>

#include "stm32f10x.h"

#include "stm32f10x_flash.h"

#include "Config.h"

#include "UartCmd.h"

#include "serial.h"

#include "Relay.h"
#include "DICA.h"

#include "Beep.h"

#include "common.h"

#include "osal_snv.h"

#include "sapp.h"

#if (IWDG_SUPPORT > 0)
#include "IWDG_Driver.h"
#endif

#include "RTC_Driver.h"

INT8U * const pParameter   =  (INT8U *)CONFIG_PARAMETER_PAGE;
//INT8U * const pIdParameter =  (INT8U *)CONFIG_DEVICE_ID_PAGE;
INT8U * const pRuntimeInfo =  (INT8U *)CONFIG_RUN_TIME_INFO_PAGE;

//INT8U * const zxxxx =  (INT8U *)CONFIG_DEVICE_ID_PAGE;

const char DataStr[]=__DATE__; 
const char TimeStr[]=__TIME__; 

//const char version[] = {"STM32F103_HANALE_V1.0"};
const char version[] = {"0.1.2.190730_94401"};

#if APP_TYPE == APP_TYPE_HYPER
const char dtype[]   = {"SHLF0710"};
#else
const char dtype[]   = {"SHLF0711"};
#endif


UINT8 Config_buff[HAL_FLASH_PAGE_SIZE];
const uint8_t LocalCfgOffset[LOCAL_CONFIG_NUM] = {PARAMETER_LOCAL1_OFF};
const uint8_t LocalCfgSize[LOCAL_CONFIG_NUM] = {PARAMETER_LOCAL1_SIZE};

uint16_t gusDeviceId;

uint8_t aDevType[NV_DTYPE_SIZE];

ADMIN_STRU gAdmin;

uint16_t gusOperMode;


TSLIB_LINEAR_CONFIG_STRU gTslibLinearCfg;


LOCAL_CONFIG_STRU gCfg;

//UINT16 sRs485QryPos = 0x0;

const char *Config_GetVersion(void)
{
    return &version[0];
}

uint8 Config_SetItem(uint16 id, uint16 len, void *buf)
{
    uint8 ucStatus = osal_snv_write(id,len,buf);
    return ucStatus;
}

uint16_t Config_Get_DevId(void)
{
    return gusDeviceId;
}

uint16_t Config_Get_OpMode(void)
{
    return gusOperMode;
}


void Config_Init(void)
{

    memset(&gAdmin,0,sizeof(gAdmin));
    
    if ( osal_snv_read( NV_DEVID_ID, sizeof ( gusDeviceId ), &gusDeviceId ) != ERROR_SUCCESS )
    {
        gusDeviceId = 0;
    }

    if ( (osal_snv_read( NV_DTYPE_ID, NV_DTYPE_SIZE, &aDevType[0]) != ERROR_SUCCESS) 
        || (0 != memcmp(aDevType,dtype,NV_DTYPE_ID)))
    {
        memcpy(aDevType,dtype,NV_DTYPE_SIZE);
    
        Config_SetItem(NV_DTYPE_ID,NV_DTYPE_SIZE,aDevType);
    }

    if ( osal_snv_read( STM32_NV_APP_OPER_MODE, sizeof ( gusOperMode ), &gusOperMode ) != ERROR_SUCCESS )
    {
        gusOperMode = 0;
    }
    
    if ( osal_snv_read( STM32_NV_APP_ADMIN, ADMIN_TEXT_LENGTH, &gAdmin.NAME[0] ) != ERROR_SUCCESS )
    {
        strcpy(gAdmin.NAME,"ADMIN");
    }

    if ( osal_snv_read( STM32_NV_APP_ADMIN+1, ADMIN_TEXT_LENGTH, &gAdmin.PSD[0] ) != ERROR_SUCCESS )
    {
        strcpy(gAdmin.PSD,"860860");
    }

    if ( osal_snv_read( STM32_NV_APP_TS_CALI, PARAMETER_TS_LINEAR_SIZE, &gTslibLinearCfg ) != ERROR_SUCCESS )
    {
        //957934 4265 29 -1940704 -14 5809 65536    
        gTslibLinearCfg.a[0] = 957934;
        gTslibLinearCfg.a[1] = 4265;
        gTslibLinearCfg.a[2] = 29;
        gTslibLinearCfg.a[3] = -1940704;
        gTslibLinearCfg.a[4] = -14;
        gTslibLinearCfg.a[5] = 5809;
        gTslibLinearCfg.a[6] = 65536;
    }

    memcpy(&gCfg,&pParameter[PARAMETER_LOCAL_BEGIN_POS],PARAMETER_LOCAL_SIZE);

    //printf("%d %d %d %d %d %d %d\r\n",gTslibLinearCfg.a[0],gTslibLinearCfg.a[1],gTslibLinearCfg.a[2],gTslibLinearCfg.a[3],gTslibLinearCfg.a[4],gTslibLinearCfg.a[5],gTslibLinearCfg.a[6]);

}


void Config_Sapp_Cmd(uint8_t cmd,uint8_t *data, uint8_t len)
{
    sbTxBuf[RPC_POS_LEN]  = len; // len for data area (NOT INCLUDE CMD0&CMD1&LEN itself)
    sbTxBuf[RPC_POS_CMD0] = RPC_SYS_APP;
    sbTxBuf[RPC_POS_CMD1] = cmd;
    
    memcpy(&sbTxBuf[RPC_POS_DAT0],data,len);

    SHZNAPP_SerialResp(sappItfPort);
}


#define CONTENT_POS 2
UINT8 Config_SaveData(UINT32 ulAddress,UINT8 *pData,UINT16 usLength)
{
#if OS_CRITICAL_METHOD == 3    /* Allocate storage for CPU status register */
    OS_CPU_SR     cpu_sr = 0;
#endif    
    UINT16 idx;

    OS_ENTER_CRITICAL();

    FLASH_Unlock();

    FLASH_ErasePage(ulAddress);

    for (idx = 0; idx <  usLength; idx = idx + 4)
    {
      FLASH_ProgramWord(ulAddress, *(uint32_t *)(pData + idx));
      ulAddress += 4;
    }
    FLASH_Lock();

    OS_EXIT_CRITICAL();

    return 0;

}

#if 0
void ConfigCalcCheckSum(UINT8 *pData,UINT8 ucLen)
{
    UINT8 iLoop;
    UINT16 iCheckSum = 0;
    for (iLoop = 0; iLoop < ucLen; iLoop++)
    {
        iCheckSum += pData[iLoop];
    }

    pData[ucLen] = (iCheckSum  >> 8)&0xff;
    pData[ucLen+1] = (iCheckSum & 0xff);
}


uint8 ConfigSetDeviceId(uint8 *pCmd,uint8 CmdLen,uint8 *pRsp,uint8 *pucRspLen)
{
    UINT8 ucPayLoadLen;

    ucPayLoadLen = DEVICE_ID_SIZE;

    memcpy(Config_buff,&pCmd[SERIAL_MESSAGE_HEADER_LENGTH],ucPayLoadLen);

    pRsp[0] = SERIAL_MSG_SIZE(ucPayLoadLen);
    pRsp[1] = CMD_CLIENT2HOST_ID_SET_REPORT;

    memcpy(&pRsp[SERIAL_MESSAGE_HEADER_LENGTH],Config_buff,ucPayLoadLen);
    
    Config_SaveData(CONFIG_DEVICE_ID_PAGE,Config_buff,ucPayLoadLen);

   *pucRspLen += SERIAL_MSG_TOTAL_SIZE(ucPayLoadLen);

   return 0;
}

uint8 ConfigGetDeviceId(uint8 *pCmd,uint8 CmdLen,uint8 *pRsp,uint8 *pucRspLen)
{
    UINT8 ucPayLoadLen;

    ucPayLoadLen = DEVICE_ID_SIZE;

    // backup parameter area
    pRsp[0] = SERIAL_MSG_SIZE(ucPayLoadLen);
    pRsp[1] = CMD_CLIENT2HOST_ID_SET_REPORT;

    memcpy(&pRsp[SERIAL_MESSAGE_HEADER_LENGTH],pIdParameter,ucPayLoadLen);

    *pucRspLen += SERIAL_MSG_TOTAL_SIZE(ucPayLoadLen);

    
    return 0;

}
#endif


int Config_SaveSerialNo(uint8_t *pucSerial)
{
    
    IWDG_Feed();

    memcpy(gCfg.cfg1.serial,pucSerial,CONFIG_SERIAL_NAME_LEN);
	
    memcpy(&Config_buff[0],pParameter,PARAMETER_SIZE);

    memcpy(&Config_buff[CONFIG_PARAMETER_LOCAL_OFFSET+LocalCfgOffset[PARAMETER_LOCAL1_OFF]],&gCfg,LocalCfgSize[PARAMETER_LOCAL1_OFF]);

    IWDG_Feed();

    Config_SaveData(CONFIG_PARAMETER_PAGE,&Config_buff[0],PARAMETER_SIZE);

	return CONFIG_SERIAL_NAME_LEN;
}

int Config_GetSerialNo(uint8_t *pucSerial)
{

    memcpy(&pucSerial[0],gCfg.cfg1.serial,CONFIG_SERIAL_NAME_LEN);
    return CONFIG_SERIAL_NAME_LEN;
}



UINT8 ConfigSetParam4Serial(uint8 *pCmd,uint8 CmdLen)
{

    memcpy(&Config_buff[CONFIG_PARAMETER_IPTA_OFFSET],&pCmd[SERIAL_MESSAGE_HEADER_LENGTH+1],PARAMETER_SERIAL_AREA_LENGTH);

    return (PARAMETER_SERIAL_AREA_LENGTH + 1);
}

UINT8 ConfigSetParam4Local(uint8 *pCmd,uint8 CmdLen,int sub)
{

    memcpy(&Config_buff[CONFIG_PARAMETER_LOCAL_OFFSET+LocalCfgOffset[sub]],&pCmd[SERIAL_MESSAGE_HEADER_LENGTH+1],LocalCfgSize[sub]);

    return (LocalCfgSize[sub] + 1);
}
UINT8 ConfigGetParam4Serial(int *offset)
{

    *offset = CONFIG_PARAMETER_IPTA_OFFSET;

    return (PARAMETER_SERIAL_AREA_LENGTH + 1);
}

UINT8 ConfigGetParam4Local(int *offset,int sub)
{

    *offset = CONFIG_PARAMETER_LOCAL_OFFSET + LocalCfgOffset[sub];

    return (LocalCfgSize[sub] + 1);
}

uint8 ConfigSetParam(uint8 *pCmd,uint8 CmdLen,uint8 *pRsp,uint8 *pucRspLen)
{
    UINT8 ucPayLoadLen;
    int offset;
    
    IWDG_Feed();
    
    memcpy(&Config_buff[0],pParameter,PARAMETER_SIZE);

    switch(pCmd[2])
    {
    case DEVICE_TYPE_SERIAL:
        ucPayLoadLen = ConfigSetParam4Serial(pCmd,CmdLen);
        ConfigGetParam4Serial(&offset);
        break;
    case DEVICE_TYPE_LOCAL:
    case DEVICE_TYPE_LOCAL+1:
    case DEVICE_TYPE_LOCAL+2:
        ucPayLoadLen = ConfigSetParam4Local(pCmd,CmdLen,pCmd[2]-DEVICE_TYPE_LOCAL);
        ConfigGetParam4Local(&offset,pCmd[2]-DEVICE_TYPE_LOCAL);
        break;
    default:
        return 0xff;
    }
    
    // backup parameter area
#ifdef CHECK_CFG    
    pRsp[2] = pCmd[2];
    pRsp[0] = SERIAL_MSG_SIZE(ucPayLoadLen);
    pRsp[1] = CMD_CLIENT2HOST_PARAMETER_SET_REPORT;
#else
    pRsp[0] = 0;
    pRsp[1] = CMD_HOST2CLIENT_PARAMETER_SET;
#endif

    memcpy(&pRsp[SERIAL_MESSAGE_HEADER_LENGTH+1],&Config_buff[offset],ucPayLoadLen);

    IWDG_Feed();

    Config_SaveData(CONFIG_PARAMETER_PAGE,&Config_buff[0],PARAMETER_SIZE);

    switch(pRsp[2])
    {
    case DEVICE_TYPE_SERIAL:
        SerialInit();
        break;
    case DEVICE_TYPE_LOCAL:
        break;
    default:
        break;
    }

    *pucRspLen += SERIAL_MSG_TOTAL_SIZE(pRsp[0]);

    return 0;

}



uint8 ConfigGetParam(uint8 *pCmd,uint8 CmdLen,uint8 *pRsp,uint8 *pucRspLen)
{
    UINT8 ucPayLoadLen;

    int offset;


    switch(pCmd[2])
    {
    case DEVICE_TYPE_SERIAL:
        ucPayLoadLen = ConfigGetParam4Serial(&offset);
        break;
    case DEVICE_TYPE_LOCAL:
        ucPayLoadLen = ConfigGetParam4Local(&offset,pCmd[2]-DEVICE_TYPE_LOCAL);
        break;
     default:
        return 0xff;
    }
    pRsp[2] = pCmd[2];
    pRsp[0] = SERIAL_MSG_SIZE(ucPayLoadLen);
    pRsp[1] = CMD_CLIENT2HOST_PARAMETER_SET_REPORT;
    
    memcpy(&pRsp[SERIAL_MESSAGE_HEADER_LENGTH+1],&pParameter[offset],ucPayLoadLen-1);

    *pucRspLen += SERIAL_MSG_TOTAL_SIZE(pRsp[0]);

    return 0;

}


/* |BLOCKLEN(2BYTES)|PATTERN LEN(1BYTE)|PATTERN|PORT(1BYTE)|TRIGTYPE(1BYTES)|RSPLEN(1BYTE)|RSPCONT|*/
uint8 ConfigRs485Ctrl(uint8 *pCmd,uint8 CmdLen,uint8 *pRsp,uint8 *pucRspLen)
{
    UINT8 *pCanMsg = (UINT8 *)&pCmd[CONTENT_POS];


    switch(pCanMsg[0])
    {
    case CMD_CMD_HOST2CLIENT_RS485_CTRL_SEND: // add an pattern
        break;
    default:
        break;
    }

    return 0;
}


void ConfigReset(void)
{
    SCB->AIRCR = 0X05FA0004; 
}

void ConfigSetTime(uint8 *pCmd,uint8 CmdLen,uint8 *pRsp,uint8 *pucRspLen)
{
   UINT8 *pCanMsg = (UINT8 *)&pCmd[CONTENT_POS];

   uint16_t year = pCanMsg[0] + 1900;

   UINT8 ucPayLoadLen = 0;
   
   UINT8 ucIdx = 0;

   RTC_Set(year,pCanMsg[1],pCanMsg[2],pCanMsg[3],pCanMsg[4],pCanMsg[5]);

   pRsp[ucIdx++] = ucPayLoadLen;
   pRsp[ucIdx++] = CMD_HOST2CLIENT_TIME_CTRL;

   *pucRspLen += SERIAL_MSG_TOTAL_SIZE(ucPayLoadLen);
   
}

void ConfigGetTime(uint8 *pCmd,uint8 CmdLen,uint8 *pRsp,uint8 *pucRspLen)
{
    UINT8 ucPayLoadLen;

    UINT8 ucIdx = 0;
    
    RTC_Get();

    ucPayLoadLen = 6;
    
    // backup parameter area
    pRsp[ucIdx++] = ucPayLoadLen;
    pRsp[ucIdx++] = CMD_HOST2CLIENT_TIME_GET;
    
    pRsp[ucIdx++] = timer.w_year - 1900;
    pRsp[ucIdx++] = timer.w_month;
    pRsp[ucIdx++] = timer.w_date;
    pRsp[ucIdx++] = timer.hour;
    pRsp[ucIdx++] = timer.min;
    pRsp[ucIdx++] = timer.sec;

    *pucRspLen += SERIAL_MSG_TOTAL_SIZE(ucPayLoadLen);
    
    
}

void ConfigAdjTime(uint8 *pCmd,uint8 CmdLen,uint8 *pRsp,uint8 *pucRspLen)
{
    UINT8 ucPayLoadLen;

    UINT8 ucIdx = 0;

    UINT8 *pCanMsg = (UINT8 *)&pCmd[CONTENT_POS];
    
    RTC_Calibrate(pCanMsg[0]);

    ucPayLoadLen = 1;
    
    // backup parameter area
    Config_buff[ucIdx++] = ucPayLoadLen;
    Config_buff[ucIdx++] = CMD_HOST2CLIENT_TIME_ADJ;
    
    Config_buff[ucIdx++] = pCanMsg[0];
    
    *pucRspLen += SERIAL_MSG_TOTAL_SIZE(ucPayLoadLen);

}



uint8 Config_Entry(uint8 *pCmd,uint8 *pRsp,uint8 CmdLen,uint8 *pucRspLen)
{
    uint8 ucRet = 0;

    //VOS_LOG(VOS_LOG_DEBUG,"%d,%d",pCmd[0],pCmd[1]); // pCmd[0] For length

    switch(pCmd[1])
    {
#if 0   
    case CMD_HOST2CLIENT_DEVICEID_SET:
        ucRet = ConfigSetDeviceId(pCmd,CmdLen,pRsp,pucRspLen);
        break;
    case CMD_HOST2CLIENT_DEVICEID_GET:
        ucRet = ConfigGetDeviceId(pCmd,CmdLen,pRsp,pucRspLen);
        break;
#endif      
    case CMD_HOST2CLIENT_PARAMETER_SET:
        ucRet = ConfigSetParam(pCmd,CmdLen,pRsp,pucRspLen);
        break;
    case CMD_HOST2CLIENT_PARAMETER_GET:
        ucRet = ConfigGetParam(pCmd,CmdLen,pRsp,pucRspLen);
        break;  
    case CMD_HOST2CLIENT_RS485_CTRL:
        ucRet = ConfigRs485Ctrl(pCmd,CmdLen,pRsp,pucRspLen);
        break;
    case CMD_HOST2CLIENT_TIME_CTRL:
        ConfigSetTime(pCmd,CmdLen,pRsp,pucRspLen);
        break;
    case CMD_HOST2CLIENT_TIME_GET:
        ConfigGetTime(pCmd,CmdLen,pRsp,pucRspLen);
        break;
    case CMD_HOST2CLIENT_TIME_ADJ:
        ConfigAdjTime(pCmd,CmdLen,pRsp,pucRspLen);
        break;
    case CMD_HOST2CLIENT_BEEP:
        MainBeepWithDuration(1);
        break;
    case CMD_HOST2CLIENT_LIGHT:
        MainAlarmWithDuration(1);
        break;
    }

    return ucRet;
}

uint8 Config_Sapp_Entry(uint8 *pCmd,uint8 *pRsp,uint8 CmdLen,uint8 *pucRspLen)
{
   
   return Config_Entry(pCmd,pRsp,CmdLen,pucRspLen);
}

