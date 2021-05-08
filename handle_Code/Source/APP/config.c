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

#include "sapp_ex.h"

#if (IWDG_SUPPORT > 0)
#include "IWDG_Driver.h"
#endif

#include "RTC_Driver.h"

const char DataStr[]=__DATE__; 
const char TimeStr[]=__TIME__; 

//const char version[] = {"STM32F103_HANALE_V1.0"};
const char version[] = {"0.1.2.210508_94401"};

#if APP_TYPE == APP_TYPE_HYPER
const char dtype[]   = {"SHLF0710"};
#else
const char dtype[]   = {"SHLF0711"};
#endif


UINT8 Config_buff[512];

uint16_t gusDeviceId;

uint8_t aDevType[NV_DTYPE_SIZE];

ADMIN_STRU gAdmin;

uint16_t gusOperMode;

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

    if ( osal_snv_read( STM32_NV_APP_SN, CONFIG_SERIAL_NAME_LEN, &gCfg.cfg1.serial[0] ) != ERROR_SUCCESS )
    {
       strcpy(gCfg.cfg1.serial,"unknow");
    }

}


#define CONTENT_POS 2

int Config_GetSerialNo(uint8_t *pucSerial)
{

    memcpy(&pucSerial[0],gCfg.cfg1.serial,CONFIG_SERIAL_NAME_LEN);
    return CONFIG_SERIAL_NAME_LEN;
}

void Config_SaveSerialNo(uint8_t *pucSerial)
{
    
    memcpy(gCfg.cfg1.serial,pucSerial,CONFIG_SERIAL_NAME_LEN);

	Config_SetItem(STM32_NV_APP_SN,CONFIG_SERIAL_NAME_LEN,&gCfg.cfg1.serial[0]);
	
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
#if (RTC_SUPPORT > 0)
    case CMD_HOST2CLIENT_TIME_CTRL:
        ConfigSetTime(pCmd,CmdLen,pRsp,pucRspLen);
        break;
    case CMD_HOST2CLIENT_TIME_GET:
        ConfigGetTime(pCmd,CmdLen,pRsp,pucRspLen);
        break;
    case CMD_HOST2CLIENT_TIME_ADJ:
        ConfigAdjTime(pCmd,CmdLen,pRsp,pucRspLen);
        break;
#endif        
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

