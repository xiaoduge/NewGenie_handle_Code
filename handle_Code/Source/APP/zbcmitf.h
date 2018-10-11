#ifndef _ZB_CMITF_H
#define _ZB_CMITF_H

enum ZB_CMITF_DEVICE_TYPE {
    ZB_CMITF_DEVICE_TYPE_INTELLIGENT_SWITCH = 0,
    ZB_CMITF_DEVICE_TYPE_VIBRATE,
    ZB_CMITF_DEVICE_TYPE_NUM,
};

enum ZB_CMITF_SWITCH_ACTION_ENUM {
    ZB_CMITF_SWITCH_ACTION_OFF = 0,
    ZB_CMITF_SWITCH_ACTION_ON,
    ZB_CMITF_SWITCH_ACTION_NUM,
};

enum ZB_CMITF_CMD_ENUM
{
    ZB_CMITF_CMD_HEART_BEAT = 0,
    ZB_CMITF_CMD_SWITCH_CTL    ,
    ZB_CMITF_CMD_VIBRATOR_CTL  ,
    ZB_CMITF_CMD_PING          ,
    ZB_CMITF_CMD_ENUM          ,
};

typedef struct
{
    uint8 ucLen; /* whole packet length ,include ucLen itself */
    uint8 ucCmd;
    uint8 ucObj;
    uint8 aucData[1];
}ZB_CMDITF_CMD_STRU;

#define ZB_CMDITF_CMD_HEAD_LENGTH (sizeof(ZB_CMDITF_CMD_STRU) - 1)

#endif
