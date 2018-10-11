#ifndef _HAL_CYTMA_H_
#define _HAL_CYTMA_H_

/*****************************************************************************
 *  Global Macros & Definitions
 *****************************************************************************/

typedef enum
{
    CYTMA_MESSAGE_IRQ = 0,
    CYTMA_MESSAGE_DELAY_CHECK,
}CYTMA_MESSAGE_ENUM;

UINT8 CYTMA_ItfProcess(Message *pMsg);

void CYTMA_Init(void);

void CYTMA_second(void);


#endif
