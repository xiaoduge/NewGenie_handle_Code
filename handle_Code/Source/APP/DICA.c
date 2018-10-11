#include    <ucos_ii.h>

#include    <cpu.h>
#include    <lib_def.h>
#include    <lib_mem.h>
#include    <lib_str.h>

#include    <string.h>

#include "stm32f10x.h"

#include "stm32f10x_pwr.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_bkp.h"

#include "memory.h"
#include "msg.h"
#include "timer.h"

#include "stm32_eval.h"

#include "app_cfg.h"

#include "serial.h"

#include "Config.h"

#include "Dica.h"

#include <string.h>


const uint8_t inputs[INPUT_NUMBER] = {
    0xFF,     
};

//--------------------------------------------------------------------------
// buttons_init
//
/// This function initializes the port, state and interrupt handler for the 
/// buttons component.
//--------------------------------------------------------------------------
void Ioctrl_input_init( void )
{
    int iLoop;

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);  

    // begin user add code here to initialize input configuration

    // end user add code here
#if (RTC_SUPPORT == 0)   
    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    
    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);
    
    RCC_LSEConfig(RCC_LSE_OFF);//关闭外部低速外部时钟信号功能 后，PC14 PC15 才可以当普通IO用。
    
    //BKP_TamperPinCmd(DISABLE);
    PWR_BackupAccessCmd(DISABLE);
#endif    

    
    for (iLoop = 0; iLoop < INPUT_NUMBER; iLoop++)
    {
        stm32_gpio_cfgpin(inputs[iLoop],MAKE_PIN_CFG(GPIO_Speed_2MHz,GPIO_Mode_IPU )); 
    }

}
UINT8 GetInputLogicStatus(UINT8 ucChannel)
{
    return stm32_gpio_get_value(inputs[ucChannel]);

}

