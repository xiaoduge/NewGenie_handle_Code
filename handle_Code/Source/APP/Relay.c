#include "relay.h"

#include "Config.h"

#include <string.h>

#include "sys_time.h"

typedef struct 
{
    uint8_t ucChl;
    sys_timeo to;

    RelayPulse_Cb cb;
    
}RELAY_PULSE_STRU;


const uint8_t relays[RELAY_NUMBER] = {
    STM32F103_GPA(11),     
    STM32F103_GPA(13),     
};

const uint8_t aRelayDefaults[RELAY_NUMBER] = 
{
0,1,
};

RELAY_PULSE_STRU aPulseCtrl[RELAY_NUMBER];


void InitRelays(void)
{
    int iLoop;
    for (iLoop = 0; iLoop < RELAY_NUMBER; iLoop++)
    {
        stm32_gpio_cfgpin(relays[iLoop],MAKE_PIN_CFG(GPIO_Speed_2MHz,GPIO_Mode_Out_PP)); 
        
        stm32_gpio_set_value(relays[iLoop],aRelayDefaults[iLoop]); // ylf:  inactive low
        
        memset(&aPulseCtrl[iLoop],0,sizeof(RELAY_PULSE_STRU));

        aPulseCtrl[iLoop].ucChl = iLoop;
    }

}

void RelayLogicCtrl(UINT8 ucChannel,UINT8 ucEnable)
{

    stm32_gpio_set_value(relays[ucChannel],ucEnable); // ylf:  inactive low

}

UINT8 GetRelayLogicStatus(UINT8 ucChannel)
{

    return !!stm32_gpio_get_value(relays[ucChannel]);

}


void RelayToggle(UINT8 ucChannel)
{
    stm32_gpio_toggle_value(relays[ucChannel]); // ylf:  inactive low
    
}

void RelayPulse_cb(void *para)
{
    RELAY_PULSE_STRU *pulse = (RELAY_PULSE_STRU *)para;
    
    RelayLogicCtrl(pulse->ucChl,FALSE);
    
    if (pulse->cb)  (pulse->cb)();
}


void RelayPulse(UINT8 ucChannel,uint32_t duration,RelayPulse_Cb cb)
{
    aPulseCtrl[ucChannel].cb = cb;

    RelayLogicCtrl(ucChannel,TRUE);

    sys_timeout(duration,SYS_TIMER_ONE_SHOT,duration,RelayPulse_cb,&aPulseCtrl[ucChannel],&aPulseCtrl[ucChannel].to);
}



