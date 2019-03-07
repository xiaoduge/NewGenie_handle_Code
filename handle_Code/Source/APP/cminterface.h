#ifndef _CM_INTERFACE_H_
#define _CM_INTERFACE_H_

#pragma   pack(1)

#define APP_PROTOL_CANID_RSVD           (0x0)
#define APP_PROTOL_CANID_ALLOC_BEGIN    (0X1)
#define APP_PROTOL_CANID_ALLOC_END      (511)
#define APP_PROTOL_CANID_DYNAMIC_BEGIN  (512)
#define APP_PROTOL_CANID_DYNAMIC_END    (1022)
#define APP_PROTOL_CANID_BROADCAST      (0x3ff)
#define APP_PROTOL_CANID_INVALID        (0xFFFF)
#define APP_PROTOL_CANID_DYNAMIC_RANGE  (APP_PROTOL_CANID_DYNAMIC_END-APP_PROTOL_CANID_DYNAMIC_BEGIN)


#define APP_PROTOL_PACKET_LEN_POS      (0)
#define APP_PROTOL_PACKET_DEV_TYPE_POS (1)
#define APP_PROTOL_PACKET_MSG_TYPE_POS (2)
#define APP_PROTOL_PACKET_CONT_POS     (3)

#define APP_PROTOL_PACKET_HEADER       (3)

#define APP_PROTOL_PACKET_RSP_MASK     (0x80)

#define APP_DEVICE_SERIAL_NUMBER_LENGTH (9)

#define APP_HAND_SET_BEGIN_ADDRESS     (0x0A)
#define APP_HAND_SET_MAX_NUMBER        (0x08)
#define APP_HAND_SET_END_ADDRESS       (APP_HAND_SET_BEGIN_ADDRESS + APP_HAND_SET_MAX_NUMBER)


#define APP_RF_READER_BEGIN_ADDRESS     (0x14)
#define APP_RF_READER_MAX_NUMBER        (0x08)
#define APP_RF_READER_END_ADDRESS       (APP_RF_READER_BEGIN_ADDRESS + APP_RF_READER_MAX_NUMBER)

#define APP_SN_LENGTH                   (10)
#define APP_CAT_LENGTH                  (9)
#define APP_LOT_LENGTH                  (10)

#define APP_EXE_VALVE_NUM      (8)
#define APP_EXE_G_PUMP_NUM     (2)
#define APP_EXE_R_PUMP_NUM     (2)
#define APP_EXE_RECT_NUM       (3)
#define APP_EXE_EDI_NUM        (1)
#define APP_EXE_HO_VALVE_NUM   (2)

#define APP_EXE_SWITCHS        (APP_EXE_VALVE_NUM + APP_EXE_G_PUMP_NUM + APP_EXE_R_PUMP_NUM + APP_EXE_RECT_NUM + APP_EXE_EDI_NUM)
#define APP_EXE_SWITCHS_MASK   ((1<<APP_EXE_SWITCHS)-1)

#define APP_EXE_E1_NO          (0)
#define APP_EXE_E2_NO          (1)
#define APP_EXE_E3_NO          (2)
#define APP_EXE_E4_NO          (3)
#define APP_EXE_E5_NO          (4)
#define APP_EXE_E6_NO          (5)
#define APP_EXE_E9_NO          (6)
#define APP_EXE_E10_NO         (7)
#define APP_EXE_EMASK          ((1<<APP_EXE_VALVE_NUM) - 1)



#define APP_EXE_C3_NO         (8)  /* G pump */
#define APP_EXE_C4_NO         (9)  /* G pump */
#define APP_EXE_C1_NO         (10) /* R pump */
#define APP_EXE_C2_NO         (11) /* R pump */
#define APP_EXE_GCMASK          ((1<<APP_EXE_G_PUMP_NUM ) - 1)
#define APP_EXE_RCMASK          ((1<<APP_EXE_R_PUMP_NUM) - 1)

#define APP_EXE_N1_NO         (12)
#define APP_EXE_N2_NO         (13)
#define APP_EXE_N3_NO         (14)
#define APP_EXE_NMASK          ((1<<APP_EXE_RECT_NUM) - 1)

#define APP_EXE_T1_NO         (15)
#define APP_EXE_TMASK          ((1<<APP_EXE_EDI_NUM) - 1)


#define APP_EXE_HO_E7_NO      (17)
#define APP_EXE_HO_E8_NO      (18)
#define APP_EXE_TESTMASK      ((1<<APP_EXE_HO_VALVE_NUM) - 1)

#define APP_EXE_INNER_SWITCHS  (APP_EXE_SWITCHS_MASK & (~((1<<APP_EXE_E10_NO)|(1<<APP_EXE_N3_NO))))


#define APP_EXE_PRESSURE_METER (3)
#define APP_EXE_PM1_NO         (0)
#define APP_EXE_PM2_NO         (1)
#define APP_EXE_PM3_NO         (2)
#define APP_EXE_PM_MASK        ((1<<APP_EXE_PRESSURE_METER)-1)

#define APP_EXE_ECO_NUM       (5)
#define APP_EXE_I1_NO         (0)
#define APP_EXE_I2_NO         (1)
#define APP_EXE_I3_NO         (2)
#define APP_EXE_I4_NO         (3)
#define APP_EXE_I5_NO         (4)
#define APP_EXE_I_MASK        ((1<<APP_EXE_ECO_NUM)-1)

#define APP_EXE_IP_MASK       (APP_EXE_I_MASK|(APP_EXE_PM_MASK<<APP_EXE_ECO_NUM))

#define APP_EXE_RECT1_NO         (0)
#define APP_EXE_RECT2_NO         (1)
#define APP_EXE_RECT3_NO         (2)
#define APP_EXE_RECT_MASK        ((1<<APP_EXE_RECT_NUM)-1)


#define APP_EXE_DIN1_NO       (0) /* k3 */
#define APP_EXE_DIN2_NO       (1) /* k4 */
#define APP_EXE_DIN3_NO       (2) /* k5 */

#define APP_EXE_DIN_NUM       (3)

#define APP_EXE_DIN_MASK      ((1<<APP_EXE_DIN_NUM)-1)
#define APP_EXE_DIN_LEAK_KEY  (APP_EXE_DIN2_NO)
#define APP_EXE_DIN_IWP_KEY   (APP_EXE_DIN3_NO)  /* Incoming water pressure */
#define APP_EXE_DIN_RF_KEY    (APP_EXE_DIN1_NO)  /* Reverse Flush key */
#define APP_EXE_DIN_FAIL_MASK (APP_EXE_DIN_MASK & (~(1 << APP_EXE_DIN_IWP_KEY)))

#define APP_FM_FLOW_METER_NUM (4)
#define APP_FM_FM1_NO         (0)
#define APP_FM_FM2_NO         (1)
#define APP_FM_FM3_NO         (2)
#define APP_FM_FM4_NO         (3)
#define APP_FM_FM_MASK        ((1<<APP_FM_FLOW_METER_NUM)-1)

#define APP_EXE_INPUT_REG_LOOP_OFFSET      (0)
#define APP_EXE_INPUT_REG_LOOP_NUM         (APP_EXE_PRESSURE_METER)

#define APP_EXE_INPUT_REG_RECTIFIER_OFFSET (APP_EXE_INPUT_REG_LOOP_OFFSET + APP_EXE_INPUT_REG_LOOP_NUM)
#define APP_EXE_INPUT_REG_RECTIFIER_NUM    (APP_EXE_RECT_NUM)

#define APP_EXE_INPUT_REG_EDI_OFFSET       (APP_EXE_INPUT_REG_RECTIFIER_OFFSET + APP_EXE_INPUT_REG_RECTIFIER_NUM)
#define APP_EXE_INPUT_REG_EDI_NUM          (APP_EXE_EDI_NUM)

#define APP_EXE_INPUT_REG_PUMP_OFFSET      (APP_EXE_INPUT_REG_EDI_OFFSET + APP_EXE_INPUT_REG_EDI_NUM)
#define APP_EXE_INPUT_REG_PUMP_NUM         (APP_EXE_G_PUMP_NUM)

#define APP_EXE_INPUT_REG_REGPUMPI_OFFSET  (APP_EXE_INPUT_REG_PUMP_OFFSET + APP_EXE_INPUT_REG_PUMP_NUM)
#define APP_EXE_INPUT_REG_REGPUMPI_NUM     (APP_EXE_R_PUMP_NUM)

#define APP_EXE_INPUT_REG_REGPUMPV_OFFSET  (APP_EXE_INPUT_REG_REGPUMPI_OFFSET + APP_EXE_INPUT_REG_REGPUMPI_NUM)
#define APP_EXE_INPUT_REG_REGPUMPV_NUM     (APP_EXE_R_PUMP_NUM)

#define APP_EXE_MAX_ECO_NUMBER             (APP_EXE_ECO_NUM)

#define APP_EXE_INPUT_REG_QMI_OFFSET       (APP_EXE_INPUT_REG_REGPUMPV_OFFSET + APP_EXE_INPUT_REG_REGPUMPV_NUM)
#define APP_EXE_INPUT_REG_QMI_NUM          (APP_EXE_MAX_ECO_NUMBER*3)

#define APP_EXE_INPUT_REG_DIN_OFFSET       (APP_EXE_INPUT_REG_QMI_OFFSET + APP_EXE_INPUT_REG_QMI_NUM)
#define APP_EXE_INPUT_REG_DIN_NUM          (1)

#define APP_EXE_MAX_INPUT_REGISTERS        (APP_EXE_INPUT_REG_LOOP_NUM + APP_EXE_INPUT_REG_RECTIFIER_NUM + APP_EXE_INPUT_REG_RECTIFIER_NUM +APP_EXE_INPUT_REG_PUMP_NUM + APP_EXE_INPUT_REG_REGPUMPI_NUM + APP_EXE_INPUT_REG_REGPUMPV_NUM + APP_EXE_INPUT_REG_QMI_NUM + APP_EXE_INPUT_REG_DIN_NUM)


#define APP_EXE_HOLD_REG_REPORT_OFFSET    (0)
#define APP_EXE_HOLD_REG_REPORT_NUM       (2)

#define APP_EXE_HOLD_REG_RPUMP_OFFSET    (2)
#define APP_EXE_HOLD_REG_RPUMP_NUM       (2)

#define APP_EXE_MAX_HOLD_REGISTERS         (4)

#define APP_EXE_ECO_REPORT_MASK            ((1<<APP_EXE_MAX_ECO_NUMBER)-1)
#define APP_EXE_PM_REPORT_MASK             (((1<<APP_EXE_PRESSURE_METER)-1) << APP_EXE_MAX_ECO_NUMBER)

#define APP_EXE_GPUMP_REPORT_POS           ((APP_EXE_VALVE_NUM ))
#define APP_EXE_RPUMP_REPORT_POS           ((APP_EXE_VALVE_NUM + APP_EXE_G_PUMP_NUM))
#define APP_EXE_RECT_REPORT_POS            ((APP_EXE_VALVE_NUM + APP_EXE_G_PUMP_NUM + APP_EXE_R_PUMP_NUM))
#define APP_EXE_EDI_REPORT_POS             ((APP_EXE_VALVE_NUM + APP_EXE_G_PUMP_NUM + APP_EXE_G_PUMP_NUM + APP_EXE_RECT_NUM))


#define APP_EXE_GPUMP_REPORT_MASK          (APP_EXE_GCMASK << APP_EXE_GPUMP_REPORT_POS)
#define APP_EXE_RPUMP_REPORT_MASK          (APP_EXE_RCMASK << APP_EXE_RPUMP_REPORT_POS)
#define APP_EXE_RECT_REPORT_MASK           (APP_EXE_NMASK  << APP_EXE_RECT_REPORT_POS)
#define APP_EXE_EDI_REPORT_MASK            (APP_EXE_TMASK  << APP_EXE_EDI_REPORT_POS)

#define APP_TOC_SAMPLES_PER_SECOND         (2)
#define APP_TOC_OXIDIZATION_SECOND         (2)
#define APP_TOC_FLUSH2_SAMPLES             (4)

#define APP_ZIGBEE_SUB_PROTOL_LENGTH       (1)

#define APP_QTW_UNIT                         (100) // ml

#define APP_EXE_INPUT_REG_FM_NUM           APP_FM_FLOW_METER_NUM

#define APP_EXE_HB_NUM                      (3)


typedef enum
{
    APP_TRX_CAN = 0, 
    APP_TRX_ZIGBEE,  
    APP_TRX_NUM,
}APP_TRX_TYPE_ENUM;

enum TANK_WATER_LEVEL_ENUM {
    TANK_WATER_LEVEL_SCALE_00 = 0,
    TANK_WATER_LEVEL_SCALE_01,
    TANK_WATER_LEVEL_SCALE_02,
    TANK_WATER_LEVEL_SCALE_03,
    TANK_WATER_LEVEL_SCALE_04,
    TANK_WATER_LEVEL_SCALE_05,
    TANK_WATER_LEVEL_SCALE_06,
    TANK_WATER_LEVEL_SCALE_07,
    TANK_WATER_LEVEL_SCALE_08,
    TANK_WATER_LEVEL_SCALE_09,
    TANK_WATER_LEVEL_SCALE_10,
    TANK_WATER_LEVEL_SCALE_NUM,
};

enum PUMP_SPEED_ENUM {
    PUMP_SPEED_00 = 0,
    PUMP_SPEED_01,
    PUMP_SPEED_02,
    PUMP_SPEED_03,
    PUMP_SPEED_04,
    PUMP_SPEED_05,
    PUMP_SPEED_06,
    PUMP_SPEED_07,
    PUMP_SPEED_08,
    PUMP_SPEED_09,
    PUMP_SPEED_10,
    PUMP_SPEED_NUM,
};

typedef enum
{
    APP_LAN_ENG,
    APP_LAN_CHN,
    APP_LAN_SPA, /*2018/01/24 modify*/
    APP_LAN_FRE,
    APP_LAN_GER, 
    APP_LAN_ITA,
    APP_LAN_NUM,   
}APP_LAN_TYPE_ENUM;


typedef enum
{
    APP_OBJ_N_PUMP,/* Normal Pump */
    APP_OBJ_R_PUMP,/* Regulator Pump*/
    APP_OBJ_VALVE, /* Magnetic Valve */ 
    APP_OBJ_RECT,  /* Rectifer */
    APP_OBJ_B,     /* Pressure Meter */
    APP_OBJ_I,     /* Sensor for Water Qulality Messure */
    APP_OBJ_S,     /* Sensor for Water Volumn Messure */
    APP_OBJ_EDI,   /* EDI */
}APP_OBJ_TYPE_ENUM;

typedef enum
{
   APP_OBJ_VALUE_U32 = 0,
   APP_OBJ_VALUE_FLOAT ,
   APP_OBJ_VALUE_CUST ,
}APP_OBJ_VALUE_ENUM;

typedef enum
{
   APP_DEV_TYPE_HOST = 0,
   APP_DEV_TYPE_MAIN_CTRL ,
   APP_DEV_TYPE_EXE_BOARD,
   APP_DEV_TYPE_FLOW_METER,
   APP_DEV_TYPE_HAND_SET,
   APP_DEV_TYPE_RF_READER,
   /*2018/03/04 add */
   APP_DEV_TYPE_EXE_COMBINED = 0x10,
   APP_DEV_TYPE_BROADCAST    = 0xff, 
}APP_DEV_TYPE_ENUM;


typedef enum
{
    APP_DEV_HS_SUB_HYPER = 0,
    APP_DEV_HS_SUB_REGULAR,
    APP_DEV_HS_SUB_NUM,
}APP_DEV_HS_SUB_TYPE_ENUM;

typedef enum
{
    APP_PAKCET_ADDRESS_HOST   = 0X00,
    APP_PAKCET_ADDRESS_MC     = 0X01 , 
    APP_PAKCET_ADDRESS_EXE    = 0X02 , 
    APP_PAKCET_ADDRESS_FM     = 0X03,
    APP_PAKCET_ADDRESS_HS     = 0X0A,
	APP_PAKCET_ADDRESS_RF	  = 0X14,
    
}APP_PACKET_ADDRESS_ENUM;


typedef enum
{
    APP_RFID_SUB_TYPE_PPACK_CLEANPACK   = 0, /* IN CLEAN STAGE CLEANPACK, OTHERWISE PPACK*/
    APP_RFID_SUB_TYPE_HPACK_ATPACK, 
    APP_RFID_SUB_TYPE_UPACK_HPACK , 
    APP_RFID_SUB_TYPE_PREPACK,
    APP_RFID_SUB_TYPE_ROPACK_OTHERS,
    APP_RFID_SUB_TYPE_NUM,
}APP_RFID_SUB_TYPE_ENUM;

typedef enum
{
    APP_WORK_MODE_NORMAL   = 0, /* IN CLEAN STAGE CLEANPACK, OTHERWISE PPACK*/
    APP_WORK_MODE_CLEAN,
    APP_WORK_MODE_INSTALL,
    APP_WORK_MODE_NUM,
}APP_WORK_MODE_ENUM;


typedef enum
{
    APP_PAKCET_COMM_ONLINE_NOTI   = 0X00,
    APP_PACKET_COMM_HEART_BEAT    = 0X01 , 
    APP_PACKET_COMM_HOST_RESET    = 0X02 , 
    APP_PACKET_COMM_ZIGBEE_IND    = 0X03 , 

    
    APP_PACKET_CLIENT_REPORT      = 0X10,
    APP_PACKET_HAND_OPERATION     = 0X11,
    APP_PACKET_STATE_INDICATION   = 0X12,
	APP_PACKET_RF_OPERATION	      = 0X13,
	APP_PACKET_EXE_OPERATION      = 0x14,
    
}APP_PACKET_ENUM;

typedef enum
{
    APP_PACKET_RPT_FM = 0,    // FLOW METER
        
}APP_PACKET_RPT_FLOW_METER_TYPE;

typedef enum
{
    APP_PACKET_RPT_ECO =0 ,    //  Electrical pod coeffience
    APP_PACKET_RPT_PM     ,    //  pressure meter report
    APP_PACKET_DIN_STATUS ,    //  DRY INPUT state
    APP_PACKET_MT_STATUS  ,    //  device state
    APP_PACKET_RPT_GPUMP  ,      
    APP_PACKET_RPT_RPUMP  ,      
    APP_PACKET_RPT_RECT   ,      
    APP_PACKET_RPT_EDI    ,       
    APP_PACKET_RPT_TOC,
        
}APP_PACKET_RPT_EXE_BOARD_TYPE;

typedef enum
{
    APP_PACKET_HS_STATE_IDLE = 0,
    APP_PACKET_HS_STATE_RUN, /* System Is Ready */
    APP_PACKET_HS_STATE_QTW, /* Quantity Taking Water */
    APP_PACKET_HS_STATE_CIR, /* Circulation */
    APP_PACKET_HS_STATE_DEC, /* Decompress */
    APP_PACKET_HS_STATE_WAIT /*wait 2019.3.5 add*/
}APP_PACKET_HS_STATE_ENUM;

typedef enum
{
    APP_PACKET_HO_CIR = 0,    //  CIRCULATION
    APP_PACKET_HO_QTW   ,    //  Taking quantified water
    APP_PACKET_HO_STA   ,    //  status
    APP_PACKET_HO_SPEED ,
    APP_PACKET_HO_QL_UPDATE,
    APP_PACKET_HO_QTW_VOLUMN ,
    APP_PACKET_HO_ALARM_STATE_NOT,
    APP_PACKET_HO_SYSTEM_TEST,
    APP_PACKET_HO_WQ_NOTIFY,
	APP_PACKET_HO_ADR_RST = 0X10,    //  reset CAN address
	APP_PACKET_HO_ADR_SET       ,	 //  SET CAN ADR
	APP_PACKET_HO_ADR_QRY       ,	 //  QRY CAN ADR
    APP_PACKET_HO_ZB_IND = 0X20,
}APP_PACKET_HAND_OPERATION_TYPE;


typedef enum
{
    APP_PACKET_HO_ERROR_CODE_SUCC = 0,
    APP_PACKET_HO_ERROR_CODE_UNREADY,
    APP_PACKET_HO_ERROR_CODE_UNSUPPORT,
    APP_PACKET_HO_ERROR_CODE_TANK_EMPTY,
    APP_PACKET_HO_ERROR_CODE_UNKNOW,
    APP_PACKET_HO_ERROR_CODE_TIMEOUT,

}APP_PACKET_HO_ERROR_CODE_ENUM;

typedef enum
{
    APP_PACKET_RF_ERROR_CODE_SUCC = 0,
    APP_PACKET_RF_NO_DEVICE,
    APP_PACKET_RF_ACCESS_CONFLICT,
    APP_PACKET_RF_ACCESS_TIMEOUT,
}APP_PACKET_RF_ERROR_CODE_ENUM;

typedef enum
{
    CIR_TYPE_UP = 0, /* UP */
    CIR_TYPE_HP,    /* TANK CIR*/
    CIR_TYPE_NUM,
}APP_CIR_TYPE_ENUM;


typedef enum
{ 
    APP_PACKET_RF_SEARCH = 0,
    APP_PACKET_RF_READ,    
    APP_PACKET_RF_WRITE   ,    
        
	APP_PACKET_RF_ADR_RST = 0X10,    //  reset CAN address
	APP_PACKET_RF_ADR_SET       ,	 //  SET CAN ADR
	APP_PACKET_RF_ADR_QRY       ,	 //  QRY CAN ADR
}APP_PACKET_RF_OPERATION_TYPE;

typedef enum
{
    APP_PACKET_RPT_RF_STATE = 0,    //  rfid state reprot
        
}APP_PACKET_RPT_RF_READER_TYPE;

typedef enum
{
    APP_PACKET_EXE_TOC = 0,    //  TOC
	APP_PACKET_EXE_NUM    ,	 
}APP_PACKET_EXE_OPERATION_TYPE;

typedef enum
{
    APP_PACKET_HO_SYS_TEST_TYPE_TEST = 0,
    APP_PACKET_HO_SYS_TEST_TYPE_DEPRESSURE, 
}APP_PACKET_HO_SYS_TEST_TYPE_ENUM;

typedef struct
{
    uint8_t ucLen;
    uint8_t ucTransId;
    uint8_t ucDevType; /* refer APP_DEV_TYPE_ENUM */
    uint8_t ucMsgType;
}APP_PACKET_COMM_STRU;

typedef struct
{
    APP_PACKET_COMM_STRU hdr;
    uint8_t acInfo[1];
}APP_PACKET_ONLINE_NOTI_IND_STRU;


typedef struct
{
    APP_PACKET_COMM_STRU hdr;
    uint8_t aucInfo[1];
}APP_PACKET_ONLINE_NOTI_CONF_STRU;

typedef struct
{
    uint16_t usHeartBeatPeriod;
}APP_PACKET_ONLINE_NOTI_CONF_EXE_STRU;

typedef APP_PACKET_ONLINE_NOTI_CONF_EXE_STRU APP_PACKET_ONLINE_NOTI_CONF_FM_STRU;

typedef APP_PACKET_ONLINE_NOTI_CONF_EXE_STRU APP_PACKET_ONLINE_NOTI_CONF_RFID_STRU;


typedef struct
{
    uint16_t usHeartBeatPeriod;
    uint8_t  ucLan;
    uint8_t  ucMode; 
    uint8_t  ucState;
    uint8_t  ucAlarmState;
    uint8_t  ucLiquidLevel;
    uint8_t  ucQtwSpeed;
    uint8_t  ucHaveTOC;
    uint8_t  ucAddData;
}APP_PACKET_ONLINE_NOTI_CONF_HANDLER_STRU;


typedef struct
{
    APP_PACKET_COMM_STRU hdr;
    uint8_t aucData[1];
}APP_PACKET_HEART_BEAT_REQ_STRU;

typedef struct
{
    APP_PACKET_COMM_STRU hdr;
    uint16_t usAddress;
}APP_PACKET_ZIGBEE_IND_STRU;


typedef struct
{
    APP_PACKET_COMM_STRU hdr;
    uint8_t ucRptType;
    uint8_t aucData[1];
}APP_PACKET_CLIENT_RPT_IND_STRU;

typedef struct
{
     uint8_t ucId;
     uint32_t ulValue;
}APP_PACKET_RPT_FM_STRU; // flow meter


typedef struct
{
    APP_PACKET_COMM_STRU hdr;
    uint8_t ucOpsType;   /* refer APP_PACKET_HAND_OPERATION_TYPE */
    uint8_t aucData[1];
}APP_PACKET_HO_STRU;

typedef struct
{
    uint8_t ucState;    /* refer: APP_PACKET_HS_STATE_ENUM */
    uint8_t ucResult;
    uint8_t ucAddData;   /* APP_PACKET_HS_STATE_QTW : TW flags;  APP_PACKET_HS_STATE_CIR : CIR TYPE */
	uint8_t ucAlarmState;
}APP_PACKET_HO_STATE_STRU;

typedef enum
{
    APP_PACKET_HO_ACTION_STOP = 0,    //  CIRCULATION
    APP_PACKET_HO_ACTION_START  ,    //  Taking quantified water
        
}APP_PACKET_HO_ACTION_TYPE;


typedef struct
{    
     uint8_t ucAction;
}APP_PACKET_HO_CIR_REQ_STRU; // flow meter


typedef struct
{    
     uint8_t ucAction;
     uint8_t ucResult;
}APP_PACKET_HO_CIR_RSP_STRU; // flow meter
typedef struct
{    
     uint8_t  ucAction;
     uint32_t ulVolumn;        //  0XFFFFFFFF for normal takeing water, other: qtw
}APP_PACKET_HO_QTW_REQ_STRU; // flow meter

typedef struct
{    
     uint8_t ucAction;
     uint8_t ucResult;
     uint8_t ucUnit;
     uint16_t usPulseNum;  /* pulses/litre */
     uint32_t ulVolumn;     /* 0XFFFFFFFF for normal takeing water, other: qtw */
     uint8_t ucType;
}APP_PACKET_HO_QTW_RSP_STRU; 

typedef APP_PACKET_HO_CIR_REQ_STRU APP_PACKET_HO_SPEED_REQ_STRU;

typedef struct
{
    uint8_t ucAction;
    uint8_t ucType;    /*APP_PACKET_HO_SYS_TEST_TYPE_ENUM*/
}APP_PACKET_HO_SYSTEMTEST_REQ_STRU;

typedef struct
{
    uint8_t ucAction;
    uint8_t ucType;    /*APP_PACKET_HO_SYS_TEST_TYPE_ENUM*/
    uint8_t ucResult;
}APP_PACKET_HO_SYSTEMTEST_RSP_STRU;


typedef struct
{    
     uint8_t ucAction;
     uint8_t ucResult;
     uint8_t ucSpeed;
     uint8_t ucType; /* APP_DEV_HS_SUB_TYPE_ENUM */
}APP_PACKET_HO_SPEED_RSP_STRU;

enum APP_PACKET_HO_QL_TYPE_ENUM
{
    APP_PACKET_HO_QL_TYPE_LEVEL = 0x1,
    APP_PACKET_HO_QL_TYPE_PPB   = 0x2,
};


typedef struct
{    
     uint8_t ucMask;             /* bit0 for ucLevel, bit1 for ppb, others reserved */
     uint8_t ucLevel;
     float   fWaterQppb;
}APP_PACKET_HO_QL_UPD_STRU;

enum APP_PACKET_HO_ALARM_TYPE_ENUM
{
    APP_PACKET_HO_ALARM_TYPE_NOT = 0x1,
    APP_PACKET_HO_ALARM_TYPE_ALM = 0x2,
};

typedef struct
{    
     uint8_t ucMask;             /* bit0 for alarm, bit1 for notify, others reserved */
}APP_PACKET_HO_ALARM_STATE_NOT_STRU;



typedef struct
{
    unsigned short usTemp;
    float          fWaterQ; /* Water Quality */
}APP_ECO_VALUE_STRU;

typedef struct
{
     uint8_t  ucId;
     APP_ECO_VALUE_STRU ev;
}APP_PACKET_RPT_ECO_STRU; // flow meter

typedef struct
{
     uint8_t  ucId;
     uint8_t  ucType;  /* refer: APP_OBJ_TYPE_ENUM */
     uint8_t  ucState;
}APP_PACKET_STATE_STRU; 

typedef struct
{
     uint8_t  ucState;
}APP_PACKET_RPT_DIN_STRU; 

typedef struct
{
    float          fB; 
    float          fP; 
}APP_PACKET_RPT_TOC_STRU; 

typedef struct
{
     uint32_t aulFm[APP_FM_FLOW_METER_NUM];
}APP_PACKET_ONLINE_NOTI_IND_FM_STRU;


typedef struct
{
     uint8_t ucHandsetAddr;
	 uint8_t aucSn[APP_DEVICE_SERIAL_NUMBER_LENGTH];
}APP_PACKET_ONLINE_NOTI_IND_HANDLE_STRU;

typedef struct
{
     uint16_t ausHoldRegs[APP_EXE_MAX_HOLD_REGISTERS];
}APP_PACKET_ONLINE_NOTI_IND_EXE_STRU;

typedef struct
{
     APP_PACKET_ONLINE_NOTI_IND_EXE_STRU exe;
     APP_PACKET_ONLINE_NOTI_IND_FM_STRU  fm;
}APP_PACKET_ONLINE_NOTI_IND_EXE_COMBINED_STRU;


typedef struct
{
    APP_PACKET_COMM_STRU hdr;
    uint8_t ucOpsType;   /* refer APP_PACKET_RF_OPERATION_TYPE */
    uint8_t aucData[1];
}APP_PACKET_RF_STRU;

typedef struct
{
	uint8_t ucLen;
    uint8_t ucOff;
}APP_PACKET_RF_READ_REQ_STRU;

typedef struct
{
	uint8_t ucLen;
    uint8_t ucOff;
    uint8_t aucData[1];
}APP_PACKET_RF_WRITE_REQ_STRU;

typedef struct
{
    uint8_t ucRslt;    
}APP_PACKET_RF_WRITE_RSP_STRU;

typedef struct
{
    uint8_t ucRslt;     
    uint8_t aucData[1];
}APP_PACKET_RF_READ_RSP_STRU;

typedef struct
{
    uint8_t ucLen;
    uint8_t ucOff;
    uint8_t aucData[1];
}APP_PACKET_RPT_RF_READ_CONT_STRU; // flow meter


typedef struct
{
     uint8_t  ucState;
     uint8_t  aucData[1];
}APP_PACKET_RPT_RF_STATE_STRU; // flow meter


typedef struct
{
     uint8_t  ucBlkNum;
     uint8_t  ucBlkSize;
     uint8_t  aucUid[8];
}APP_PACKET_RPT_RF_STATE_CONT_STRU; // flow meter


typedef struct
{
    APP_PACKET_COMM_STRU hdr;
    uint8_t ucOpsType;   /* refer APP_PACKET_RF_OPERATION_TYPE */
}APP_PACKET_RF_SEARCH_REQ_STRU;


typedef struct
{
	uint8_t ucLen;
    uint8_t aucData[1];
}APP_PACKET_RF_SEARCH_RSP_STRU;

typedef enum
{
    APP_PACKET_EXE_TOC_STAGE_FLUSH1 = 0,    //  FLUSH 1
    APP_PACKET_EXE_TOC_STAGE_OXDIZATION  ,  //  oxidize
    APP_PACKET_EXE_TOC_STAGE_FLUSH2      ,  //  FLUSH2
    APP_PACKET_EXE_TOC_STAGE_NUM         ,  // also used for exit TOC stage
}APP_PACKET_EXE_TOC_STAGE_ENUM;


/* TOC State */
typedef struct
{    
     uint8_t ucStage; /* refer APP_PACKET_EXE_TOC_STAGE_ENUM */
}APP_PACKET_EXE_TOC_REQ_STRU; 


typedef struct
{
    uint8_t ucRslt;    
}APP_PACKET_EXE_TOC_RSP_STRU;


typedef APP_PACKET_RPT_FM_STRU         APP_PACKET_RPT_RPUMP_STRU;
typedef APP_PACKET_RPT_FM_STRU         APP_PACKET_RPT_GPUMP_STRU;
typedef APP_PACKET_RPT_FM_STRU         APP_PACKET_RPT_EDI_STRU;
typedef APP_PACKET_RPT_FM_STRU         APP_PACKET_RPT_PM_STRU;
typedef APP_PACKET_RPT_FM_STRU         APP_PACKET_RPT_RECT_STRU;
typedef APP_PACKET_HEART_BEAT_REQ_STRU APP_PACKET_HEART_BEAT_RSP_STRU ;
typedef APP_PACKET_HEART_BEAT_REQ_STRU APP_PACKET_HOST_RESET_STRU ;
typedef APP_PACKET_HO_STRU             APP_PACKET_STATE_IND_STRU;
typedef APP_PACKET_HO_STRU             APP_PACKET_EXE_STRU;

#define APP_PROTOL_HEADER_LEN (sizeof(APP_PACKET_COMM_STRU))

#define APP_POROTOL_PACKET_ONLINE_NOTI_IND_TOTAL_LENGTH (sizeof(APP_PACKET_ONLINE_NOTI_IND_STRU))

#define APP_POROTOL_PACKET_ONLINE_NOTI_IND_PAYLOAD_LENGTH (APP_POROTOL_PACKET_ONLINE_NOTI_IND_TOTAL_LENGTH - APP_PROTOL_HEADER_LEN)

#define APP_POROTOL_PACKET_ONLINE_NOTI_CNF_TOTAL_LENGTH (sizeof(APP_PACKET_ONLINE_NOTI_CONF_STRU)-1)

#define APP_POROTOL_PACKET_ONLINE_NOTI_CNF_PAYLOAD_LENGTH (APP_POROTOL_PACKET_ONLINE_NOTI_CNF_TOTAL_LENGTH - APP_PROTOL_HEADER_LEN)

#define APP_POROTOL_PACKET_HEART_BEAT_RSP_TOTAL_LENGTH (sizeof(APP_PACKET_HEART_BEAT_RSP_STRU))

#define APP_POROTOL_PACKET_HEART_BEAT_RSP_PAYLOAD_LENGTH (APP_POROTOL_PACKET_HEART_BEAT_RSP_TOTAL_LENGTH - APP_PROTOL_HEADER_LEN)

#define APP_POROTOL_PACKET_HEART_BEAT_REQ_TOTAL_LENGTH (sizeof(APP_PACKET_HEART_BEAT_REQ_STRU))

#define APP_POROTOL_PACKET_HEART_BEAT_REQ_PAYLOAD_LENGTH (APP_POROTOL_PACKET_HEART_BEAT_REQ_TOTAL_LENGTH - APP_PROTOL_HEADER_LEN)

#define APP_POROTOL_PACKET_HOST_RESET_TOTAL_LENGTH (sizeof(APP_PACKET_HOST_RESET_STRU))

#define APP_POROTOL_PACKET_HOST_RESET_PAYLOAD_LENGTH (APP_POROTOL_PACKET_HOST_RESET_TOTAL_LENGTH - APP_PROTOL_HEADER_LEN)

#define APP_POROTOL_PACKET_CLIENT_RPT_IND_TOTAL_LENGTH (sizeof(APP_PACKET_CLIENT_RPT_IND_STRU) - 1)

#define APP_POROTOL_PACKET_CLIENT_RPT_IND_PAYLOAD_LENGTH (APP_POROTOL_PACKET_CLIENT_RPT_IND_TOTAL_LENGTH - APP_PROTOL_HEADER_LEN)

#define APP_POROTOL_PACKET_HO_COMMON_LENGTH (sizeof(APP_PACKET_HO_STRU) - 1)

#define APP_POROTOL_PACKET_RF_COMMON_LENGTH (sizeof(APP_PACKET_RF_STRU) - 1)

typedef enum
{
    MODBUS_FUNC_CODE_Read_Coil_Status = 0X1,
    MODBUS_FUNC_CODE_Read_Input_Status = 0X2,
    MODBUS_FUNC_CODE_Read_Holding_Registers = 0X3,
    MODBUS_FUNC_CODE_Read_Input_Registers   = 0X4,
    MODBUS_FUNC_CODE_Force_Single_Coil = 0x5,
    MODBUS_FUNC_CODE_Preset_Single_Register = 0x6,
    MODBUS_FUNC_CODE_Force_Multiple_Coils = 0xF,
    MODBUS_FUNC_CODE_Preset_Multiple_Registers = 0x10,
    MODBUS_FUNC_CODE_Mask_Write_0X_Register    = 0x80,
    MODBUS_FUNC_CODE_Mask_Write_4X_Register    = 0x81,

}MODBUS_FUNC_CODE_ENUM;


typedef struct
{
    uint8_t ucAddress;
    uint8_t ucFuncode;
    uint8_t aucData[1];
}MODBUS_PACKET_COMM_STRU;

#define MODBUS_PACKET_COMM_HDR_LEN (2)

#define INVALID_FM_VALUE (0xffffffff)


#pragma pack()

#endif

