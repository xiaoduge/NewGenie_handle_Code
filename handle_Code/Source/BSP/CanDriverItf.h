#ifndef _CAN_DRIVER_ITF_H_
#define _CAN_DRIVER_ITF_H_

#include "DtypeStm32.h"

typedef struct can_frame {
    unsigned int  can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
    unsigned char can_dlc; /* data length code: 0 .. 8 */
    unsigned char data[8];
}CAN_FRAME;

typedef enum
{
    CAN_ITF_RATE_500kbps   = 0 ,      
    CAN_ITF_RATE_250kbps,   
    CAN_ITF_RATE_125kbps,      
    CAN_ITF_RATE_62_5kbps,     
    CAN_ITF_RATE_31_25kbps ,   
    CAN_ITF_RATE_NUM ,   
}CAN_ITF_RATE_ENUM;



 /* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U /* error frame */
 
 /* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU /* omit EFF, RTR, ERR flags */

#endif
