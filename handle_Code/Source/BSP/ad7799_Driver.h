#ifndef _AD7799_DRIVER_H_
#define _AD7799_DRIVER_H_

/**
  ******************************************************************************
  * @file    ad7799_driver.h 
  * @author  STM32 Team
  * @version V1.0.0
  * @date    10/10/2014
  * @brief   Interface for AD7799 BSP access.
  ******************************************************************************
  *          DO NOT MODIFY ANY OF THE FOLLOWING CODE EXCEPT UNDER PERMISSION OF ITS PROPRIETOR
  * @copy
  *
  *
  * <h2><center>&copy; COPYRIGHT 2012 Shanghai ZhenNeng Corporation</center></h2>
  */ 
typedef enum
{
    AD7799_WADDR_COMM        = 0X0,
    AD7799_WADDR_MODE        = 0X1,
    AD7799_WADDR_CONFIG      = 0X2,
    AD7799_WADDR_DATA        = 0X3,
    AD7799_WADDR_ID          = 0X4,
    AD7799_WADDR_IO          = 0X5,
    AD7799_WADDR_OFFSET      = 0X6,
    AD7799_WADDR_FULL_SCALE  = 0X7,
       
}AD7799_WADDR_EUNM;

typedef enum
{
    AD7799_RADDR_STATUS      = 0X0,
    AD7799_RADDR_MODE        = 0X1,
    AD7799_RADDR_CONFIG      = 0X2,
    AD7799_RADDR_DATA        = 0X3,
    AD7799_RADDR_ID          = 0X4,
    AD7799_RADDR_IO          = 0X5,
    AD7799_RADDR_OFFSET      = 0X6,
    AD7799_RADDR_FULL_SCALE  = 0X7,
       
}AD7799_RADDR_EUNM;

typedef enum
{
    AD7799_CH0        = 0X0, // AIN1(+) 每 AIN1(每)
    AD7799_CH1        = 0X1, // AIN2(+) 每 AIN2(每)
    AD7799_CH2        = 0X2, // AIN3(+) 每 AIN3(每)
    AD7799_CH3        = 0X3, // AIN1(每) 每 AIN1(每) 
    AD7799_CH_RSVD1   = 0X4, // Reserved
    AD7799_CH_RSVD2   = 0X5, // Reserved/
    AD7799_CH_RSVD3   = 0X6, // Reserved
    AD7799_AVDD_MONI  = 0X7, // AVDD monitor
}AD7799_CHL_EUNM;

typedef enum
{
    AD7799_GAIN0        = 0X0, // 2.5 V
    AD7799_GAIN1        = 0X1, // 1.25 V
    AD7799_GAIN2        = 0X2, // 625 mV
    AD7799_GAIN3        = 0X3, // 312.5 mV
    AD7799_GAIN4        = 0X4, // 156.2 mV
    AD7799_GAIN5        = 0X5, // 78.125 mV
    AD7799_GAIN6        = 0X6, // 39.06 mV
    AD7799_GAIN7        = 0X7, // 19.53 mV
}AD7799_GAIN_EUNM;


typedef enum
{
    AD7799_MOD0        = 0X0, // Continuous-Conversion Mode (Default).
    AD7799_MOD1        = 0X1, // Single-Conversion Mode
    AD7799_MOD2        = 0X2, // Idle Mode
    AD7799_MOD3        = 0X3, // Power-Down Mode
    AD7799_MOD4        = 0X4, // Internal Zero-Scale Calibration
    AD7799_MOD5        = 0X5, // Internal Full-Scale Calibration
    AD7799_MOD6        = 0X6, // System Zero-Scale Calibration.
    AD7799_MOD7        = 0X7, // System Full-Scale Calibration. 
}AD7799_MODE_EUNM;

typedef enum
{
    AD7799_UR0        = 0X0, // Reserved
    AD7799_UR1        = 0X1, // 470 hz
    AD7799_UR2        = 0X2, // 242
    AD7799_UR3        = 0X3, // 123
    AD7799_UR4        = 0X4, // 62
    AD7799_UR5        = 0X5, // 50
    AD7799_UR6        = 0X6, // 39.
    AD7799_UR7        = 0X7, // 33.2. 
    AD7799_UR8        = 0X8, // 19.6. 
    AD7799_UR9        = 0X9, // 16.7. 
    AD7799_UR10       = 0X10, // 16.7. 
    AD7799_UR11       = 0X11, // 12.5. 
    AD7799_UR12       = 0X12, // 10. 
    AD7799_UR13       = 0X13, // 8.33. 
    AD7799_UR14       = 0X14, // 6.25. 
    AD7799_UR15       = 0X15, // 4.17. 
}AD7799_UR_EUNM;

    
#define IO_REG_ENABLE (1<< 6)
#define IO_REG_5      (1<< 5)
#define IO_REG_4      (1<< 4)
    
#define CON_REG_UNIPOL (1<<4)
#define CON_REG_BO     (1<<5)
    
#define CON_REG_BUFFER (1<<4)
#define CON_REG_REF_DET (1<<5)
    
#define AD7799_MAKE_REG(reg) (reg<<3)

void sAD7799_DeInit(void);
uint8_t sAD7799_SendByte(uint8_t byte);
uint8_t sAD7799_ReadByte(void);

void sAD7799_Write_Bytes(uint8_t *data,uint8_t len);
void sAD7799_Read_Bytes(uint8_t *data,uint8_t len);
void sAD7799_Init(void);
void sAD7799_ReadReg(uint8_t RegAddr,uint8_t *Buffer,uint8_t Length);  
void sAD7799_WriteReg(uint8_t RegAddr,uint8_t *Buffer,uint8_t Length);  
uint8_t sAD7799_WaitBusy(void) ; 
int sAD7799_Install_WaitBusy_Callback(dica_event_callback handle,void *para);
void sAD7799_Cs(int enable);
uint8_t sAD7799_GetBusyState(void);  
uint8_t sAD7799_Calibrate(uint8_t CHx,uint8_t Gain) ; 
void sAD7799_Reset(void);

#endif
