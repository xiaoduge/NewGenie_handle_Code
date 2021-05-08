#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "memory.h"
#include "msg.h"
#include "timer.h"

#include "App_cfg.h"

#include "sapp_ex.h"

#include "common.h"
#include "cminterface.h"

// common for all

#define MAX_DEVICE_ID_LENGTH 4

#define DEVICE_ID_SIZE MAX_DEVICE_ID_LENGTH

#define CAN_ID_SIZE 2

#define VERSION_SIZE (2)

#define BRD_TYPE_SIZE (1)

// 
enum
{
    DEVICE_TYPE_LOCAL = 0,
    DEVICE_BUTT,
};

enum
{
    STM32_NV_APP_ADMIN       = 0xA0,
	STM32_NV_APP_OPER_MODE   = 0xB0,
    STM32_NV_APP_SN          = 0xB8,
    STM32_NV_APP_TS_CALI     = 0xC0,

};

#define CONFIG_LAST_FLASH_PAGE_ADDRESS (HAL_FLASH_BEGIN+HAL_NV_PAGE_END*HAL_FLASH_PAGE_SIZE)

#define CMD_HOST2CLIENT_LIGHT 0XF0   //
#define CMD_HOST2CLIENT_BEEP 0xF1    // 
#define CMD_HOST2CLIENT_QUERY_PIN 0xF4    //
#define CMD_HOST2CLIENT_BUZZ 0xF6        // 
#define CMD_HOST2CLIENT_RESET 0xF7        // 
#define CMD_HOST2CLIENT_MOTOR_CTRL 0xFA        // Pulse Oximeter Command
#define CMD_HOST2CLIENT_CAN_CTRL 0xFB        // local simulating can testing
#define CMD_HOST2CLIENT_RS485_CTRL 0xFD // ird configuration bin downloading
        #define CMD_CMD_HOST2CLIENT_RS485_CTRL_SEND 0
        #define CMD_CMD_HOST2CLIENT_RS485_CTRL_GET  1

#define CMD_HOST2CLIENT_PARAMETER_SET 0xD0        // 
#define CMD_HOST2CLIENT_PARAMETER_GET 0xD1        // 

#define CMD_HOST2CLIENT_DEVICEID_SET 0xD2        // 
#define CMD_HOST2CLIENT_DEVICEID_GET 0xD3        // 

#define CMD_HOST2CLIENT_CANID_SET 0xD4        // 
#define CMD_HOST2CLIENT_CANID_GET 0xD5        // 

#define CMD_HOST2CLIENT_DEVICE_RESET 0xD6     // RESET
#define CMD_HOST2CLIENT_FORCE_BOOT   0xd7


#define CMD_HOST2CLIENT_TIME_CTRL 0xC0        // 
#define CMD_HOST2CLIENT_TIME_GET  0xC1        // 
#define CMD_HOST2CLIENT_TIME_ADJ  0xC2        // 

#define CMD_CLIENT2HOST_REPORT 0X00  //管脚状态上报
#define CMD_CLIENT2HOST_PARAMETER_SET_REPORT 0X10  //管脚状态上报
#define CMD_CLIENT2HOST_ID_SET_REPORT 0X11  //设备ID上报
#define CMD_CLIENT2HOST_CANID_SET_REPORT 0X12  //设备ID上报

#define CMD_CLIENT2HOST_TUNNEL_REPORT 0X80  //设备ID上报

// for stm32 in little endian
typedef struct
{
    UINT32 BaundRate;
    UINT8  ucDataBits;
    UINT8  ucStopBits;
    UINT8  ucParity;
    UINT8  ucIdleCheckThd;
    UINT16 usRsv;
}SERIAL_CONFIG_STRU;

#define CONFIG_MAX_SERIAL_PORTS (2)
#define PARAMETER_SERIAL_BEGIN_POS (CONFIG_PARAMETER_IPTA_OFFSET)  // 
#define PARAMETER_SERIAL_LAST_POS  (PARAMETER_SERIAL_BEGIN_POS+sizeof(SERIAL_CONFIG_STRU))
#define PARAMETER_SERIAL_SIZE      (PARAMETER_SERIAL_LAST_POS-PARAMETER_SERIAL_BEGIN_POS)

/* CURRENT TOTAL LENGTH 20 BYTES*/
#define PARAMETER_SERIAL_AREA_LENGTH \
(PARAMETER_SERIAL_SIZE*CONFIG_MAX_SERIAL_PORTS)


#define CONFIG_SERIAL_NAME_LEN (APP_SN_LENGTH)

typedef struct
{
    UINT32 language; // DONT CARE
    UINT32 devType;  // DONT CARE
	char   serial[CONFIG_SERIAL_NAME_LEN+1];
}LOCAL_CONFIG1_STRU;

typedef struct {
	LOCAL_CONFIG1_STRU cfg1;

}LOCAL_CONFIG_STRU;

#define ADMIN_TEXT_LENGTH (16)

typedef struct
{
    char NAME[ADMIN_TEXT_LENGTH+1]; // late implement
    char PSD[ADMIN_TEXT_LENGTH+1];

}ADMIN_STRU;

typedef struct
{
    int a[7];
}TSLIB_LINEAR_CONFIG_STRU;

#define PARAMETER_TS_LINEAR_SIZE (sizeof(TSLIB_LINEAR_CONFIG_STRU))


#define LOCAL_CONFIG_NUM (1)


#define INVALID_CONFIG_VALUE (0XFFFF)

#define INVALID_CONFIG_VALUE_LONG (0XFFFFFFFF)

#define SERIAL_MESSAGE_HEADER_LENGTH (2)
#define SERIAL_MESSAGE_CMD_LENGTH (1)

#define SERIAL_MESSAGE_EXTRA_LENGTH (SERIAL_MESSAGE_HEADER_LENGTH)

//
#define SERIAL_MSG_SIZE(ucPayLoad)(SERIAL_MESSAGE_CMD_LENGTH+ucPayLoad)

#define SERIAL_MSG_BODY_SIZE(ucPayLoad)(SERIAL_MESSAGE_HEADER_LENGTH+ucPayLoad)

#define SERIAL_MSG_TOTAL_SIZE(ucPayLoad)(SERIAL_MESSAGE_EXTRA_LENGTH+ucPayLoad)


typedef enum
{
    BAUD_DATA_8BITS = 0,
    BAUD_DATA_9BITS  ,
    BAUD_DATA_BITS_NUM,

}ENUM_BAUD_DATA_BITS;

typedef enum
{
    BAUD_STOP_1BITS = 0,
    BAUD_STOP_1_5_BITS  ,
    BAUD_STOP_2_BITS ,
    BAUD_STOP_BITS_NUM,

}ENUM_BAUD_STOP_BITS;

typedef enum
{
    BAUD_PARITY_NO = 0,
    BAUD_PARITY_EVEN  ,
    BAUD_PARITY_ODD ,
    BAUD_PARITY_NUM,
}ENUM_BAUD_PARITY;

extern UINT8 Config_buff[512];
extern UINT8 aDevType[NV_DTYPE_SIZE];

extern uint16_t gusDeviceId;
extern ADMIN_STRU gAdmin;
extern uint16 gusOperMode;

extern LOCAL_CONFIG_STRU gCfg;

UINT8 Config_SaveData(UINT32 ulAddress,UINT8 *pData,UINT16 usLength);

const char *Config_GetVersion(void);

uint16_t Config_Get_DevId(void);

void Config_Init(void);

uint8_t Config_SetItem(uint16_t id, uint16_t len, void *buf);

uint8_t Config_Sapp_Entry(uint8_t *pCmd,uint8_t *pRsp,uint8_t CmdLen,uint8_t *pucRspLen);

void Config_Sapp_Cmd(uint8_t cmd,uint8_t *data, uint8_t len);

void Config_SaveParam(LOCAL_CONFIG_STRU *pLocalCfg);
void Config_SaveSerialNo(uint8_t *pucSerial);
int Config_GetSerialNo(uint8_t *pucSerial);

#endif
