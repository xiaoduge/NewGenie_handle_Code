#ifndef _DICA_H_
#define _DICA_H_


#include "stm32_eval.h"

#include "DtypeStm32.h"

typedef enum
{
   INPUT_DUMMY       = 0,  // INPUT FOR TANK WATER LEVEL CHECK
   INPUT_NUMBER,
}INPUT_ENUM;

void Ioctrl_input_init( void );
UINT8 GetInputLogicStatus(UINT8 ucChannel);

#endif
