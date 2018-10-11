#include "Beep.h"

#include "App_Cfg.h"

#ifdef RUN_PIN
#undef RUN_PIN
#endif

#define RUN_PIN (STM32F103_GPA(14))

#define ALARM_PIN (STM32F103_GPA(15))

#define BEEP_PIN (0xFF)

void AlarmInit(void)
{
#if (ALARM_SUPPORT > 0)

    stm32_gpio_cfgpin(ALARM_PIN,MAKE_PIN_CFG(GPIO_Speed_2MHz,GPIO_Mode_Out_PP)); 
    
    stm32_gpio_set_value(ALARM_PIN,1); // ylf:  output low

#endif    
}

void Alarm(int On)
{
#if (ALARM_SUPPORT > 0)
    if (On)
    {
        stm32_gpio_set_value(ALARM_PIN,0); // ylf:  output low
    }
    else
    {
        stm32_gpio_set_value(ALARM_PIN,1); // ylf:  output low
    }
#endif    
}

void AlarmToggle(void)
{
#if (ALARM_SUPPORT > 0)
    stm32_gpio_toggle_value(ALARM_PIN);
#endif
}


/*********************************************

  ucDuration:  Time duration of BEEP (UNIT 100MS), 
                    Range (0.1s ~25.5s)

                    0: for stop
*********************************************/
void MainAlarmWithDuration(UINT8 ucDuration)
{

    if (0 == ucDuration)
    {
        Alarm(0);
        return ;
    }
    Alarm(1);
    
    AddTimer(TIMER_CHECK_ALARM,OS_TMR_OPT_ONE_SHOT,(ucDuration*100)/(1000/OS_TMR_CFG_TICKS_PER_SEC),0);
}


void BeepInit(void)
{
#if (BEEP_SUPPORT > 0)

    stm32_gpio_cfgpin(BEEP_PIN,MAKE_PIN_CFG(GPIO_Speed_2MHz,GPIO_Mode_Out_PP)); 
    
    stm32_gpio_set_value(BEEP_PIN,0); // ylf:  output low

#endif    
}

void Beep(int On)
{
#if (BEEP_SUPPORT > 0)

    if (On)
    {
        stm32_gpio_set_value(BEEP_PIN,1); // ylf:  output low
    }
    else
    {
        stm32_gpio_set_value(BEEP_PIN,0); // ylf:  output low
        
    }
#endif    
}

void BeepToggle(void)
{
#if (BEEP_SUPPORT > 0)
    stm32_gpio_toggle_value(BEEP_PIN);
#endif    

}


/*********************************************

  ucDuration:  Time duration of BEEP (UNIT 100MS), 
                    Range (0.1s ~25.5s)

                    0: for stop
*********************************************/
void MainBeepWithDuration(UINT8 ucDuration)
{

    if (0 == ucDuration)
    {
        Beep(0);
        return ;
    }
    Beep(1);
    
    AddTimer(TIMER_CHECK_BEEP,OS_TMR_OPT_ONE_SHOT,(ucDuration*100)/(1000/OS_TMR_CFG_TICKS_PER_SEC),0);
}


void RunInit(void)
{
#if (RUN_SUPPORT > 0)

    stm32_gpio_cfgpin(RUN_PIN,MAKE_PIN_CFG(GPIO_Speed_2MHz,GPIO_Mode_Out_PP)); 
    
    stm32_gpio_set_value(RUN_PIN,1); // ylf:  output low

#endif    
}

void Run(int On)
{
#if (RUN_SUPPORT > 0)
    if (On)
    {
        stm32_gpio_set_value(RUN_PIN,0); // ylf:  output low
    }
    else
    {
        stm32_gpio_set_value(RUN_PIN,1); // ylf:  output low
    }
#endif    
}

void RunToggle(void)
{
#if (RUN_SUPPORT > 0)
    stm32_gpio_toggle_value(RUN_PIN);
#endif
}

