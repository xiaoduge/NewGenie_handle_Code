#ifndef  _SD_DRIVER_HAL_
#define _SD_DRIVER_HAL_

#define SPI_SPEED_LOW                    0
#define SPI_SPEED_HIGH                   1


#define     SD_HardWareInit( )  SD_LowLevel_Init( )
#define     SPI_SetHighSpeed( ) SD_LowLevel_SetSpeed(TRUE )
#define     SPI_SetLowSpeed() SD_LowLevel_SetSpeed(FALSE )

#define     SPI_SendByte( x )   SPI_ReadWriteByte( x )
#define     SPI_RecByte( )      SPI_ReadWriteByte( 0xFF )


void SD_LowLevel_DeInit(void);

void SD_LowLevel_Init(void);

void SD_LowLevel_SetSpeed(unsigned char SpeedSet);

unsigned char SPI_ReadWriteByte(unsigned char TxData);

void SD_CS_Assert(void)  ;  

void SD_CS_Deassert(void)   ;


#endif
