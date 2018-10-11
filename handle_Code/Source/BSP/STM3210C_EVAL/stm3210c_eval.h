/**
  ******************************************************************************
  * @file    stm3210c_eval.h
  * @author  MCD Application Team
  * @version V4.2.0
  * @date    04/16/2010
  * @brief   This file contains definitions for STM3210C_EVAL's Leds, push-buttons
  *          COM ports, SD Card on SPI and sEE on I2C hardware resources.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 210 STMicroelectronics</center></h2>
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM3210C_EVAL_H
#define __STM3210C_EVAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_spi.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_can.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"


/** @addtogroup Utilities
  * @{
  */

/** @addtogroup STM32_EVAL
  * @{
  */
    
/** @addtogroup STM3210C_EVAL
  * @{
  */ 

/** @addtogroup STM3210C_EVAL_LOW_LEVEL
  * @{
  */ 
  
/** @defgroup STM3210C_EVAL_LOW_LEVEL_Exported_Types
  * @{
  */
/**
  * @}
  */ 

/** @defgroup STM3210C_EVAL_LOW_LEVEL_Exported_Constants
  * @{
  */ 

/** @addtogroup STM3210C_EVAL_LOW_LEVEL_LED
  * @{
  */
#define LEDn                             LED_NUM     // yulunfeng

#define RUN_PIN                         GPIO_Pin_2
#define RUN_GPIO_PORT                   GPIOD
#define RUN_GPIO_CLK                    RCC_APB2Periph_GPIOD  

/** @addtogroup STM3210C_EVAL_LOW_LEVEL_COM
  * @{
  */
#ifdef STM32F10X_HD     
#define COMn                             5
#else
#define COMn                             3
#endif

/**
 * @brief Definition for COM port1, connected to USART1
 */ 
#define EVAL_COM1_STR                    "USART1"
#define EVAL_COM1                        USART1
#define EVAL_COM1_CLK                    RCC_APB2Periph_USART1
#define EVAL_COM1_TX_PIN                 GPIO_Pin_9
#define EVAL_COM1_TX_GPIO_PORT           GPIOA
#define EVAL_COM1_TX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define EVAL_COM1_RX_PIN                 GPIO_Pin_10
#define EVAL_COM1_RX_GPIO_PORT           GPIOA
#define EVAL_COM1_RX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define EVAL_COM1_IRQn                   USART1_IRQn

   
/**
 * @brief Definition for COM port1, connected to USART2 (USART2 pins remapped on GPIOD)
 */ 
#define EVAL_COM2_STR                    "USART2"
#define EVAL_COM2                        USART2
#define EVAL_COM2_CLK                    RCC_APB1Periph_USART2
#define EVAL_COM2_TX_PIN                 GPIO_Pin_2
#define EVAL_COM2_TX_GPIO_PORT           GPIOA
#define EVAL_COM2_TX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define EVAL_COM2_RX_PIN                 GPIO_Pin_3
#define EVAL_COM2_RX_GPIO_PORT           GPIOA
#define EVAL_COM2_RX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define EVAL_COM2_IRQn                   USART2_IRQn

   


/**
 * @brief Definition for COM port3, connected to USART3 (YLF ADD)
 */ 
#define EVAL_COM3_STR                    "USART3"
#define EVAL_COM3                        USART3
#define EVAL_COM3_CLK                    RCC_APB1Periph_USART3
#define EVAL_COM3_TX_PIN                 GPIO_Pin_10
#define EVAL_COM3_TX_GPIO_PORT           GPIOB
#define EVAL_COM3_TX_GPIO_CLK            RCC_APB2Periph_GPIOB
#define EVAL_COM3_RX_PIN                 GPIO_Pin_11
#define EVAL_COM3_RX_GPIO_PORT           GPIOB
#define EVAL_COM3_RX_GPIO_CLK            RCC_APB2Periph_GPIOB
#define EVAL_COM3_IRQn                   USART3_IRQn

/**
 * @brief Definition for COM port4, connected to UART4 (YLF ADD)
 */ 
#ifdef STM32F10X_HD     
 
#define EVAL_COM4_STR                    "UART4"
#define EVAL_COM4                        UART4
#define EVAL_COM4_CLK                    RCC_APB1Periph_UART4
#define EVAL_COM4_TX_PIN                 GPIO_Pin_10
#define EVAL_COM4_TX_GPIO_PORT           GPIOC
#define EVAL_COM4_TX_GPIO_CLK            RCC_APB2Periph_GPIOC
#define EVAL_COM4_RX_PIN                 GPIO_Pin_11
#define EVAL_COM4_RX_GPIO_PORT           GPIOC
#define EVAL_COM4_RX_GPIO_CLK            RCC_APB2Periph_GPIOC
#define EVAL_COM4_IRQn                   UART4_IRQn

/**
 * @brief Definition for COM port5, connected to UART5 (YLF ADD)
 */ 
#define EVAL_COM5_STR                    "UART5"
#define EVAL_COM5                        UART5
#define EVAL_COM5_CLK                    RCC_APB1Periph_UART5
#define EVAL_COM5_TX_PIN                 GPIO_Pin_12
#define EVAL_COM5_TX_GPIO_PORT           GPIOC
#define EVAL_COM5_TX_GPIO_CLK            RCC_APB2Periph_GPIOC
#define EVAL_COM5_RX_PIN                 GPIO_Pin_2
#define EVAL_COM5_RX_GPIO_PORT           GPIOD
#define EVAL_COM5_RX_GPIO_CLK            RCC_APB2Periph_GPIOD
#define EVAL_COM5_IRQn                   UART5_IRQn
#endif
/**
  * @}
  */
   
/**
  * @}
  */ 
  
/** @defgroup STM3210C_EVAL_LOW_LEVEL_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM3210C_EVAL_LOW_LEVEL_Exported_Functions
  * @{
  */ 
void STM_EVAL_LEDInit(Led_TypeDef Led);
void STM_EVAL_LEDOn(Led_TypeDef Led);
void STM_EVAL_LEDOff(Led_TypeDef Led);
void STM_EVAL_LEDToggle(Led_TypeDef Led);
int STM_EVAL_GetLEDStatus(Led_TypeDef Relay);

/**
  * @}
  */
  
#ifdef __cplusplus
}
#endif

#endif /* __STM3210C_EVAL_H */
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */
    
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
