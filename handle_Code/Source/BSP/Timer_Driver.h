#ifndef _TIMER_DRIVER_H_
#define _TIMER_DRIVER_H_

typedef enum 
{
  TIMER1 = 0, // YLF: FOR RUN
  TIMER2 ,
  TIMER3 ,
  TIMER4 ,
  TIMER_NUM
} Timer_TypeDef;


typedef int (*tim_event_callback)(int Tim,int event,void *para);

void TIM_Init_CMP(Timer_TypeDef Tim);
void TIM_Init_PWM(Timer_TypeDef Tim,uint32_t ulIoRemap,int freq,int duty);
void TIM_SwitchOuputMode(Timer_TypeDef Tim,int chl,int mode);
void TIM_Init_General(Timer_TypeDef Tim,int freq);
void TIM_event_handler(unsigned char channel,unsigned int event);
void TIM_InstallHandler(unsigned char channel,unsigned int event,tim_event_callback handle,void *para);
void TIM_UninstallHandler(unsigned char channel);
void TIM_PwmStart(Timer_TypeDef Tim,uint32_t ulIoRemap,int hz);
void TIM_PwmStop(Timer_TypeDef Tim);
void TIM_Init_ETR(Timer_TypeDef Tim,uint32_t ulIoRemap);

#endif
