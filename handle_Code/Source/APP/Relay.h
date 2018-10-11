#ifndef _RELAY_H_
#define _RELAY_H_

#include "stm32_eval.h"

#include "DtypeStm32.h"

typedef enum
{
   RELAY_VALVE_1       = 0,  //  
   RELAY_ZIGBEE_RESET,
   RELAY_NUMBER,
}RELAY_ENUM;

typedef void (*RelayPulse_Cb)(void);


void InitRelays(void);
void RelayLogicCtrl(UINT8 ucChannel,UINT8 ucEnable);
UINT8 GetRelayLogicStatus(UINT8 ucChannel);
void RelayToggle(UINT8 ucChannel);
void RelayPulse(UINT8 ucChannel,uint32_t duration,RelayPulse_Cb cb);

#endif
