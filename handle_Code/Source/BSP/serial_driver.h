#ifndef _SERIAL_DRIVER_H_
#define _SERIAL_DRIVER_H_

typedef enum 
{
  COM1 = 0, // usart 1, for stdio
  COM2 = 1, // usart 2, for RS485(or rs232)
  COM3 = 2, // usart 3, for RS485(or rs232)

#ifdef STM32F10X_HD  
  COM4 = 3, // usart 4, for RS485(or rs232)
  COM5 = 4, // usart 5, for RS485(or rs232)
#endif
  
} COM_TypeDef;   

void STM_EVAL_COMInit(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct);

#endif
