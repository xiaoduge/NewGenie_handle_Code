#ifndef _HAL_SPI_DRIVER_H_
#define _HAL_SPI_DRIVER_H_

typedef enum 
{
  HAL_SPI0 = 0, // YLF: FOR RUN
  HAL_SPI1 ,
  HAL_NUM
} HAL_TypeDef;

struct spi_adpater
{
    uint8_t (*send)(uint8_t ucByte);

    uint8_t (*read)(void);
};

struct spi_adpater *hal_spi_Init(int spi);

#endif


