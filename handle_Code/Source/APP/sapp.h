#ifndef _SAPP_H_
#define _SAPP_H_

#include "hal_types.h"

#include "hal_rpc.h"

#define SAPP_BUF_SIZE                 196
#define SAPP_MAX_SIZE                (SAPP_BUF_SIZE - RPC_FRAME_HDR_SZ - RPC_UART_FRAME_OVHD)

// when responding - this is probably not necessary for smooth functionality.
#define SAPP_RSP_MASK                 0x80

// Status codes
#define SAPP_SUCCESS                  0
#define SAPP_FAILURE                  1
#define SAPP_INVALID_FCS              2
#define SAPP_INVALID_FILE             3
#define SAPP_FILESYSTEM_ERROR         4
#define SAPP_ALREADY_STARTED          5
#define SAPP_NO_RESPOSNE              6
#define SAPP_VALIDATE_FAILED          7
#define SAPP_CANCELED                 8
#define SAPP_IGNORED                  9

// Indices into the RPC data (RPC_POS_DAT0):
#define SAPP_REQ_ADDR_LSB             RPC_POS_DAT0
#define SAPP_REQ_ADDR_MSB            (SAPP_REQ_ADDR_LSB+1)
#define SAPP_REQ_DAT0                (SAPP_REQ_ADDR_MSB+1)
#define SAPP_RSP_STATUS               RPC_POS_DAT0
#define SAPP_RSP_ADDR_LSB            (SAPP_RSP_STATUS+1)
#define SAPP_RSP_ADDR_MSB            (SAPP_RSP_ADDR_LSB+1)
#define SAPP_RSP_DAT0                (SAPP_RSP_ADDR_MSB+1)
#define SAPP_READ_HDR_LEN            (SAPP_RSP_DAT0 - SAPP_RSP_STATUS)

typedef enum
{
    SAPP_CMD_RESET    = 0,
    SAPP_CMD_SYS_INIT    , // restore system setting
    
    SAPP_CMD_AT       = 0x20,
    SAPP_CMD_DATA     = 0x21,
    SAPP_CMD_CFG      = 0x22,
    SAPP_CMD_AIR_AT   = 0x23, // only valid for COO
    SAPP_CMD_AIR_RSP  = 0x24, // only valid for COO
    SAPP_CMD_AIR_DATA = 0x25, // coo <->other
    
    SAPP_CMD_MODBUS   = 0x28, 

    SAPP_CMD_DBG_INFO = 0x30,

    //user defined 
    SAPP_CMD_USER     = 0X40,

    // Innder comm
    SAPP_CMD_INNER    = 0X50,

    SAPP_CMD_DATA_EXT = 0x60, // extension for SAPP_CMD_DATA
    
    
        
}SAPP_CMD_ENUM;

typedef enum
{
    SAPI_CMD_DEV_STATE = 0x0,  /* zb state report */
    SAPI_CMD_DATA      = 0x21, /* same as SAPP_CMD_DATA for compatible reason*/
        
}SAPI_CMD_ENUM;


typedef enum
{
    // Commands to Bootloader
    SBL_WRITE_CMD                = 0x01,
    SBL_READ_CMD                 = 0x02,
    SBL_ENABLE_CMD               = 0x03,
    SBL_HANDSHAKE_CMD            = 0x04,
    SBL_QUERY_ID_CMD             = 0x05,
    SBL_QUERY_VERSION_CMD        = 0x06,
    SBL_SET_ADDR_CMD             = 0x07,
    SBL_SET_BOOTP_CMD            = 0x08,
    SBL_FILE_SIZE_CMD            = 0x09,
    
    // Commands to Target Application
    SBL_TGT_BOOTLOAD             = 0x10,  // Erase the image valid signature & jump to bootloader.

	// for User Serial Number Management
    SBL_SET_SERIAL_NO           = 0x20,
    SBL_QUERY_SERIAL_NO         = 0x21,
	
}SAPP_CMD_BOOT_ENUM;

typedef enum
{
  rpcSteSOF,
  rpcSteLen,
  rpcSteData,
  rpcSteFcs
} rpcSte_t;

typedef enum
{
    Interface_RS232 = 0,
    Interface_CAN ,
}Interface_t;

#define SAPP_PAYLOAD_LEN(len) (len-RPC_FRAME_HDR_SZ)

extern uint8 *const sbRxBuf;
extern uint8 *const sbTxBuf;
extern uint8 sbLen;
extern uint8 sappFlags;
extern uint8 sappItfType;
extern uint8 sappItfPort;
extern uint8 sappTgtType;

uint8 SHZNAPP_CanParse(uint8 *data,uint16 len);
uint8 SHZNAPP_SerialAppProc(void);
uint8 SHZNAPP_SerialBootProc(void);
uint8 SHZNAPP_SerialUnknowProc(void);
uint8 SHZNAPP_SerialResp(uint8 ucPort);
uint8 SHZNAPP_SerialParse(uint8 ucPort);
void SHZNAPP_SetTgtType(uint8 ucType);
void SHZNAPP_SerialInit(void);

#endif
