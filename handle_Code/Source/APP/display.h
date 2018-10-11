#ifndef _DISPLAY_H_
#define _DISPLAY_H_

//#define TOC
//#define UF_FUNCTION


#include "stm32_eval.h"
#include "DtypeStm32.h"
#include "config.h"
#include "sys_time.h "
#include "touch.h"
#include "cminterface.h"

#define TIME_FUNCTION

typedef enum
{
   DISPLAY_PAGE_IDLE,
   DISPLAY_PAGE_RUN,
   DISPLAY_PAGE_CIRCULATION,
   DISPLAY_PAGE_NORMAL_TAKING_WATER,
   DISPLAY_PAGE_QUANTITY_TAKING_WATER,
   DISPLAY_PAGE_USER_SET,
   DISPLAY_PAGE_ENG_SET,
   DISPLAY_PAGE_DECPRE,
   DISPLAY_PAGE_BUTT,
}DISPLAY_PAGE_ENUM;


typedef enum
{
   DISPLAY_SUB_PAGE_IDLE,

   // SUB PAGE FOR IDLE
   

   // SUB PAGE FOR USER_SET = 0X10
   // SUB PAGE FOR ENG_SET = 0X20
   // SUB PAGE FOR CIRCULATION = 0X30
   // SUB PAGE FOR NORMAL_TAKING_WATER = 0X40
   // SUB PAGE FOR QUANTITY_TAKING_WATER
   DISPLAY_SUB_PAGE_QUANTITY_TAKING_WATER_SETTING = 0X50,
   DISPLAY_SUB_PAGE_QUANTITY_TAKING_WATER_VOID_TANK ,
   DISPLAY_SUB_PAGE_QUANTITY_TAKING_WATER_TAKING ,
   // SUB PAGE FOR DECOMPRESSION
   // SUB PAGE FOR US SETTING
   
   // SUB PAGE FOR UF WASH
   // DISPLAY_SUB_PAGE_US_UF_WASH = 0X70,

   
   
   DISPLAY_SUB_PAGE_BUTT,
}DISPLAY_SUB_PAGE_ENUM;

typedef struct 
{
    uint16_t left,top,right,bottom;
}RECT;

typedef enum
{
    DISPLAY_STATE_IDLE,
    DISPLAY_STATE_CIRCULATION,
    DISPLAY_STATE_NORMAL_TAKING_WATER,
    DISPLAY_STATE_QUANTITY_TAKING_WATER,
    DISPLAY_STATE_DECOMPRESSION, 
    DISPLAY_STATE_BUTT,
        
}DISPLAY_STATE_ENUM;

typedef enum
{
    DISPLAY_SUB_STATE_IDLE = 0,

    DISPLAY_SUB_STATE_IDLE_FLUSH,

    DISPLAY_SUB_STATE_DEGASS1,

    DISPLAY_SUB_STATE_DEGASS2,

    DISPLAY_SUB_STATE_TOC_FLUSH,
    
    DISPLAY_SUB_STATE_TOC_OXIDATION,

    DISPALY_SUB_STATE_UF_PRECLEAN,
    
    DISPALY_SUB_STATE_UF_CLEAN,

    DISPLAY_SUB_STATE_BUTT,       
}DISPLAY_SUB_STATE_ENUM;

typedef enum
{
    DISP_CANITF_CODE_LOST_HB = 0, /* lost heart beat */

}DISP_CANITF_CODE_ENUM;


#define DISP_DECIMAL (1) // 10^1

#define DISP_MAX_WATER_VOLUME (1000)

#define DISP_MAX_WATER_SPEED (PUMP_SPEED_NUM - 1)

#define DISP_30SECOND2MS   (30*1000)

#define DISP_MINUTE2MS     (60*1000)

#define DISP_DAYININSECOND (24*60*60)

enum
{
    USER_TYPE_ADMIN = 0,
    USER_TYPE_MANAGER ,
    USER_TYPE_USER ,
};

typedef struct
{
    char          Name[16];
    char          Pwd[16];
    char          attr;      //0,1,2
    unsigned int  data;
}STORAGE_USER_INFO;


typedef struct 
{
    uint16_t usDummy;
}DISP_SPECIAL_IDLE;

typedef struct 
{
    uint16_t usDummy;
}DISP_SPECIAL_CIRCULATION;

typedef struct 
{
    uint16_t usDummy;
}DISP_SPECIAL_NORMAL_TAKING_WATER;

#define DISP_CHECK_PERIOD (100) // 100ms

typedef struct 
{
    uint16_t usDummy;
    uint32_t time4tw;
    uint32_t timeElaps;
    
}DISP_SPECIAL_QUANTITY_TAKING_WATER;

typedef struct 
{
    uint16_t usDummy;
}DISP_SPECIAL_DECOMPRESSION;

// add for V2
typedef enum
{
    DISPLAY_UF_SUB_STATE_1 = 0,
    DISPLAY_UF_SUB_STATE_2,
    DISPLAY_UF_SUB_STATE_3,
    DISPLAY_UF_SUB_STATE_4,
    DISPLAY_UF_SUB_STATE_5,

    DISPLAY_UF_SUB_STATE_BUTT,
        
}DISPLAY_UF_SUB_STATE_ENUM;


typedef struct
{
    uint8_t ucMain;// refer DISPLAY_PAGE_ENUM
    uint8_t ucSub; // DISPLAY_SUB_PAGE_ENUM
}PAGE_STRU;

typedef struct
{
    uint8_t ucMain;// refer DISPLAY_STATE_ENUM
    uint8_t ucSub; // DISPLAY_SUB_STATE_ENUM
}STATE_STRU;

#define DISP_K4_US_SET_TIMES (8)

#define DISP_K4_ENG_SET_TIMES (16)

typedef enum
{
    DISP_ALARM_PRODUCT_RS,
    DISP_ALARM_PRODUCT_RS1,
    DISP_ALARM_TEMPERATURE,
    DISP_ALARM_TEMPERATURE1,
    DISP_ALARM_TANK_EMPTY,
    DISP_ALARM_NUM,
}DISP_ALARM_ENUM;

#define DISPLAY_MAX_LINE_PER_PAGE (9)


typedef enum
{
    DISPLAY_SENSOR_1 = 0,
    DISPLAY_SENSOR_2,
    DISPLAY_SENSOR_3,
    DISPLAY_SENSOR_4,
    DISPLAY_SENSOR_5,
    DISPLAY_SENSOR_NUM,

}DISPLAY_SENSOR_ENUM;

#define DISPLAY_MAX_SENSOR (DISPLAY_SENSOR_NUM)


typedef enum
{
    DISP_US_ITEM_PACK = 0,
    DISP_US_ITEM_FILTER,
    DISP_US_ITEM_UV,
    DISP_US_ITEM_FLOW,
    DISP_US_ITEM_UNIT,
    DISP_US_ITEM_LANGUAGE,
    DISP_US_ITEM_SERIAL_NO,
    DISP_US_ITEM_NUM,    
}DISP_US_ITEM_ENUM;

typedef enum
{
    DISP_ES_ITEM_CELL_CONSTANT1 = 0,
    DISP_ES_ITEM_TEMP_CONSTANT1,
    DISP_ES_ITEM_RECIRCULATION,
    DISP_ES_ITEM_NUM,    
}DISP_ES_ITEM_ENUM;

typedef enum
{
    DISP_SHOW_ALARM_NONE = 0,
    DISP_SHOW_ALARM_CM ,
    DISP_SHOW_ALARM_SENSOR ,
    DISP_SHOW_ALARM_UF_CLEAN,
}DISP_SHOW_ALARM_ENUM;

#define DISP_USER_NAME_LENGTH (24)
#define DISP_USER_PASS_LENGTH (6)

typedef struct
{
    uint32_t ulBgnFm;
    uint32_t ulCurFm;
}DISP_FM_STRU;

typedef struct
{

    uint32_t bit1Admin        : 1;


    uint8_t ucTrxMap;  /* refer APP_TRX_TYPE_ENUM */
    
    uint8_t ucDspWelcomeTime;

    
    STATE_STRU CurState; // refer DISPLAY_STATE_ENUM
    STATE_STRU TgtState; // refer DISPLAY_STATE_ENUM


    PAGE_STRU curPage;
    PAGE_STRU tgtPage;

    uint16_t usIdleTimes;
    uint16_t usIdleLcdTimes;

    uint16_t usSeconds;

    uint16_t usForColor; // for general purpose, i.e. setting
    uint16_t usBackColor; // for general purpose, i.e. setting

    uint16_t usBackColor4Set;

    uint8_t ucLan;
    uint8_t ucUnit;
    uint8_t ucDevType;  /* device type ,refer APP_DEV_HS_SUB_TYPE_ENUM */

    LOCAL_CONFIG_STRU *cfg;

    uint32_t ulStartSecond;

    uint16_t usQtwWq; // water quantity , unit 100ml
    uint8_t  ucQtwSpeed;
    uint8_t  bQtwFlag;

    union {
        DISP_SPECIAL_IDLE idle;
        DISP_SPECIAL_CIRCULATION cir;
        DISP_SPECIAL_NORMAL_TAKING_WATER ntw;
        DISP_SPECIAL_QUANTITY_TAKING_WATER qtw;
        DISP_SPECIAL_DECOMPRESSION dec;
    }spec;


    uint8_t   KeyMask;
    uint8_t   KeyLong;
    uint8_t   KeyState[2]; // max eight keys

    uint8_t   ucOperRslt;

    uint8_t   ucOperState; /* APP_PACKET_HS_STATE_ENUM */

    uint16_t  usPulseNum;

    uint8_t   ucLiquidLevel;

    float     fWaterPPB;
    float     fWaterQuality;
    int       iTemperature;

    uint8_t   ucAlarmState;

    uint8_t   ucBtnState;

    APP_ECO_VALUE_STRU I4,I5;

    DISP_FM_STRU Fm;

    uint8_t  ucTrxIndex;

    uint8_t ucTwType;

    uint8_t  ucHaveTOC;

}DISPLAY_STRU;

extern DISPLAY_STRU gDisplay;

void Disp_DisplayHandleIndConf(uint8_t ucTrxIndex,APP_PACKET_ONLINE_NOTI_CONF_STRU *pHoMsg);
void Disp_DisplayHandleOpsEntry(uint8_t ucTrxIndex,APP_PACKET_HO_STRU *pHoMsg,uint8_t ucDedicated);
void Disp_DisplayHandleOpsTimeout(uint8_t ucOpType);
void Disp_ClientReportEntry(APP_PACKET_CLIENT_RPT_IND_STRU *pClientRptMsg);
void Disp_DisplayHeartBeatNotify(APP_PACKET_HO_STATE_STRU *pHoState);
void Disp_zbResetInd(void);

void Disp_Init(void);
void Disp_KeyHandler(int key,int state);
void Disp_msg_Handler(Message *Msg);
void Disp_SecondTask(void);
void Disp_TouchHandler(TOUCH_EVENT *event);

#endif
