/**
  ******************************************************************************
  * @file    USART/Printf/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
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
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_tim.h"

//#include "stm32_eval.h"
#include <stdio.h>
#include <string.h>
#include <ucos_ii.h>

#include "memory.h"

#include "task.h"

#include "UartCmd.h"

#include "serial.h"

#include "stm32_eval.h"

#include "Timer_Driver.h"

#include "Beep.h"

#include "dica.h"


//extern void Decrement_TimingDelay(void);
//extern void I2S_CODEC_DataTransfer(void);

//extern  __IO uint32_t TimingDelay;
//extern ITStatus SPI_I2S_GetITStatus(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);
/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup USART_Printf
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

__ASM void HardFault_Handler_a(void)
{	
	IMPORT HardFault_Handler_C
    TST LR, #4
    ITE EQ
    MRSEQ R0, MSP
    MRSNE R0, PSP
    B HardFault_Handler_C
}


void HardFault_Handler_C(unsigned int* hardfault_args)
{
	printf("\r\nsp =0x%.8X\r\n",hardfault_args);
    printf("R0    = 0x%.8X\r\n",hardfault_args[0]);         
    printf("R1    = 0x%.8X\r\n",hardfault_args[1]);         
    printf("R2    = 0x%.8X\r\n",hardfault_args[2]);         
    printf("R3    = 0x%.8X\r\n",hardfault_args[3]);         
    printf("R12   = 0x%.8X\r\n",hardfault_args[4]);         
    printf("LR    = 0x%.8X\r\n",hardfault_args[5]);         
    printf("PC    = 0x%.8X\r\n",hardfault_args[6]);         
    printf("PSR   = 0x%.8X\r\n",hardfault_args[7]);         
    printf("BFAR  = 0x%.8X\r\n",*(unsigned int*)0xE000ED38);
    printf("CFSR  = 0x%.8X\r\n",*(unsigned int*)0xE000ED28);
    printf("HFSR  = 0x%.8X\r\n",*(unsigned int*)0xE000ED2C);
    printf("DFSR  = 0x%.8X\r\n",*(unsigned int*)0xE000ED30);
    printf("AFSR  = 0x%.8X\r\n",*(unsigned int*)0xE000ED3C);
    printf("SHCSR = 0x%.8X\r\n",SCB->SHCSR);                
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  HardFault_Handler_a();

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  //static uint32_t TimingDelay = 0;
  /* Decrement the TimingDelay variable */
  //Decrement_TimingDelay();
  //TimingDelay++;
  	OS_CPU_SR  cpu_sr;

    OS_ENTER_CRITICAL();  // /* Tell uC/OS-II that we are starting an ISR*/
    if (OSRunning)
        OSIntNesting++;
    OS_EXIT_CRITICAL();	  //

    OSTimeTick();         /* Call uC/OS-II's OSTimeTick(), refer os_core.c*/

    OSIntExit();          // os_core.c
}

/*******************************************************************************
* Function Name  : SPI2_IRQHandler
* Description    : This function handles SPI2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI2_IRQHandler(void)
{
 // if ((SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_TXE) == SET))
  //{
  //  /* Send data on the SPI2 and Check the current commands */
  //  I2S_CODEC_DataTransfer();
  ///}
}

/*******************************************************************************
* Function Name  : SPI1_IRQHandler
* Description    : This function handles SPI1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI1_IRQHandler(void)
{
  //static int IRQcounter = 0;

 // if ((SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) == SET))
  //{
  //
  //}
}



void  USB_HP_CAN1_TX_IRQHandler(void)
{
#if (CAN_SUPPORT > 0)

    void SndCanData(void);

    if(SET == CAN_GetITStatus(CAN1,CAN_IT_RQCP0))
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_RQCP0);
        //CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE);
        
        // do someting
        SndCanData();
    }

    if(SET == CAN_GetITStatus(CAN1,CAN_IT_RQCP1))
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_RQCP1);
        //CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE);
        
        // do someting
        SndCanData();
    }

    if(SET == CAN_GetITStatus(CAN1,CAN_IT_RQCP2))
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_RQCP2);
        //CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE);

        // do someting
        SndCanData();
    }
#endif    

}


void  USB_LP_CAN1_RX0_IRQHandler(void)
{
//  u32 i;
#if  CAN_SUPPORT
    CanRxMsg msg;

    while(CAN_MessagePending(CAN1,CAN_FIFO0))
    {
        Message *pMsg;

        CAN_Receive(CAN1, CAN_FIFO0, (CanRxMsg *)&msg);
        
        // do someting here
        pMsg = MessageAlloc(PID_CAN,sizeof(CanRxMsg));
        if (pMsg)
        {
          pMsg->msgHead.nRcvPid = PID_MAIN_TASK;

          memcpy(pMsg->data,&msg,sizeof(CanRxMsg));
          
          if (MessageSend(pMsg))
          {
              //printf("OV+1.0\r\n");
          }
        }
        else
        {
          //printf("OV+1.1\r\n");
        }         
    }
#endif

#if  USB_SUPPORT

    {
        __USBCDC_ISR();
    }    
#endif    

}
  
void  CAN1_RX1_IRQHandler(void)
{
#if  CAN_SUPPORT
    CanRxMsg msg;

    // u32 i;
    //CanRxMsg RxMessage;
    while(CAN_MessagePending(CAN1,CAN_FIFO1))
    {
        Message *pMsg;

         CAN_Receive(CAN1, CAN_FIFO1, &msg);
    
         // do someting here
         pMsg = MessageAlloc(PID_CAN,sizeof(CanRxMsg));
         if (pMsg)
         {
             pMsg->msgHead.nRcvPid = PID_MAIN_TASK;

             memcpy(pMsg->data,&msg,sizeof(CanRxMsg));
             
             if (MessageSend(pMsg))
             {
                 //printf("OV-1.0\r\n");
             }
         }
         else
         {
             //printf("OV-1.1\r\n");
         }
      
    }   
#endif     

}

void  CAN1_SCE_IRQHandler(void)
{
}
static void USART_IRQCommHandler(int iPort)
{
 
    if(USART_GetITStatus(Serial[iPort].UsartDef, USART_IT_RXNE) != RESET)
    {
      UINT8 ucRcvData;
      /* Read one byte from the receive data register */
      ucRcvData = USART_ReceiveData(Serial[iPort].UsartDef);
#if (SERIAL_SUPPORT > 0)
      Serial_FillRcvBuf(iPort,&ucRcvData,1);
#else
      ucRcvData = ucRcvData;
#endif
  
    }

#if (SERIAL_SUPPORT > 0)
    if(USART_GetITStatus(Serial[iPort].UsartDef, USART_IT_TXE) != RESET)
    {   

      if(SERIAL_EMPTY(Serial[iPort].usSndFront, Serial[iPort].usSndRear))
      {
        /* Disable the USART Transmit interrupt */
        USART_ITConfig(Serial[iPort].UsartDef, USART_IT_TXE, DISABLE);

      }
      else
      {
          USART_SendData(Serial[iPort].UsartDef, Serial[iPort].SndBuff[Serial[iPort].usSndRear]);
          Serial[iPort].usSndRear = (Serial[iPort].usSndRear + 1)%SERIAL_MAX_SEND_BUFF_LENGTH;

          if (RS485 == Serial[iPort].ucPortType)
          {
              if(SERIAL_EMPTY(Serial[iPort].usSndFront, Serial[iPort].usSndRear))
              {
                 USART_ITConfig(Serial[iPort].UsartDef, USART_IT_TC, ENABLE);
              }
          }
      }
    }

    
    if (RS485 == Serial[iPort].ucPortType)
    {
        if (USART_GetITStatus(Serial[iPort].UsartDef, USART_IT_TC) != RESET)
        {
            if(SERIAL_EMPTY(Serial[iPort].usSndFront, Serial[iPort].usSndRear))
            {
               SerialEnableTx(iPort,FALSE); 
               
               USART_ITConfig(Serial[iPort].UsartDef, USART_IT_TC, DISABLE);
            }
        }
    }
    
#endif      

}

void USART1_IRQHandler(void)
{
#ifdef USE_USART1_STDIO
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
      UINT8 ucRcvData;
      /* Read one byte from the receive data register */
      ucRcvData = USART_ReceiveData(USART1);
      
      {
          UartCmdPutData(ucRcvData);        
      }
    }
#else
    USART_IRQCommHandler(SERIAL_PORT0);
#endif
}

/**
* @brief  This function handles USART2 global interrupt request.
* @param  None
* @retval None
*/

void USART2_IRQHandler(void)
{
  USART_IRQCommHandler(SERIAL_PORT1);
}

/**
  * @brief  This function handles USART3 global interrupt request.
  * @param  None
  * @retval None
  */
  
void USART3_IRQHandler(void)
{
    USART_IRQCommHandler(SERIAL_PORT2);
}

#ifdef STM32F10X_HD
  void  UART4_IRQHandler(void)
  {
     USART_IRQCommHandler(SERIAL_PORT3);
  }
  
  void  UART5_IRQHandler(void)
  {
     USART_IRQCommHandler(SERIAL_PORT4);
  }
#endif

#if (RTC_SUPPORT > 0)
  void RTC_IRQHandler(void)
{                               
    if(RTC->CRL & RTC_FLAG_SEC)//Second interrupt
    {        
        // RTC_Get();// update current time,let application to process
    }
    if(RTC->CRL & RTC_FLAG_ALR)//alarm interrupt
    {
          RTC->CRL&=~(RTC_FLAG_ALR);//clean inter
          // do someting ,if any
      }                                                
      RTC->CRL &= 0X0FFA;         // clean overflow & second inter
      while(!(RTC->CRL&(1<<5)));//等待RTC寄存器操作完成                                            
  }
#endif
  
void TIM1_UP_IRQHandler()
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)  
    {  
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  
          
        TIM_event_handler(TIMER4,TIM_IT_Update);      
    }   
}

void TIM1_CC_IRQHandler()
{
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)  
    {  
        uint16_t capture;
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);  
          
        capture = TIM_GetCapture1(TIM1);  
        TIM_SetCompare1(TIM1, capture + 1000);      

        TIM_event_handler(TIMER4,TIM_IT_CC1);      

     }        

}


void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

      // User Add callback functions here
      TIM_event_handler(TIMER1,TIM_IT_Update);      
  }

  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)  
  {  
      TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);  
      TIM_event_handler(TIMER1,TIM_IT_CC1);      
  }  
  
  if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)  
  {  
      TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);  
      TIM_event_handler(TIMER1,TIM_IT_CC2);      
   } 
  
  if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)  
  {  
      TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);  
      TIM_event_handler(TIMER1,TIM_IT_CC3);      
  } 

}

void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    TIM_event_handler(TIMER2,TIM_IT_Update);      
  }

   if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)  
   {  
       TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);  
       TIM_event_handler(TIMER2,TIM_IT_CC1);      
   }  
   
   if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)  
   {  
       TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);  
       TIM_event_handler(TIMER2,TIM_IT_CC2);      
   } 
   
   if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)  
   {  
       TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);  
       TIM_event_handler(TIMER2,TIM_IT_CC3);      
   } 
  
}

void TIM4_IRQHandler(void)
{

  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    
    TIM_event_handler(TIMER3,TIM_IT_Update);      
  }
  if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)  
  {  
      TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);  
      TIM_event_handler(TIMER3,TIM_IT_CC1);      
  }  
  
  if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)  
  {  
      TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);  
      
      TIM_event_handler(TIMER3,TIM_IT_CC2);      
  } 
  
  if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)  
  {  
      TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);  
      
      TIM_event_handler(TIMER3,TIM_IT_CC3);      
  } 
  
}


/**
* @brief  This function handles External lines 9 to 5 interrupt request.
* @param  None
* @retval None
*/
void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line5) != RESET) // ylf: pending inter
	{
	  /* Clear the EXTI Line 5 */
	  EXTI_ClearITPendingBit(EXTI_Line5);
      sensor_event_handler(5);
      
	}

	if (EXTI_GetITStatus(EXTI_Line6) != RESET)
	{

	  /* Clear the EXTI Line 6 */
	  EXTI_ClearITPendingBit(EXTI_Line6);
      sensor_event_handler(6);
	}

	if (EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
	  /* UP function */
	  /* Clear the EXTI Line 7 */
	  EXTI_ClearITPendingBit(EXTI_Line7);
      sensor_event_handler(7);
	}
	
	if (EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
	  /* UP function */
	  /* Clear the EXTI Line 8 */
	  EXTI_ClearITPendingBit(EXTI_Line8);
      sensor_event_handler(8);
	}

    if (EXTI_GetITStatus(EXTI_Line9) != RESET)
    {
      /* UP function */
      /* Clear the EXTI Line 9 */
      EXTI_ClearITPendingBit(EXTI_Line9);
      sensor_event_handler(9);
    }

	//MainAlarmWithDuration(1);

}

/**
* @brief  This function handles External lines 9 to 5 interrupt request.
* @param  None
* @retval None
*/
void EXTI15_10_IRQHandler(void)
{

	if (EXTI_GetITStatus(EXTI_Line10) != RESET)
	{

	  /* Clear the EXTI Line 10 */
	  EXTI_ClearITPendingBit(EXTI_Line10);
      
      sensor_event_handler(10);
      
	}

	if (EXTI_GetITStatus(EXTI_Line11) != RESET)
	{
	  /* UP function */
	  /* Clear the EXTI Line 11 */
	  EXTI_ClearITPendingBit(EXTI_Line11);
      sensor_event_handler(11);
      
	}
	
	if (EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
	  /* UP function */
	  /* Clear the EXTI Line 13 */
	  EXTI_ClearITPendingBit(EXTI_Line12);
      
      sensor_event_handler(12);
	}
    
	if (EXTI_GetITStatus(EXTI_Line13) != RESET)
	{
      
	  /* Clear the EXTI Line 14*/
	  EXTI_ClearITPendingBit(EXTI_Line13);
	  /* UP function */
      sensor_event_handler(13);
	}
    
	if (EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
	  /* Clear the EXTI Line 15 */
	  EXTI_ClearITPendingBit(EXTI_Line14);
	  /* UP function */
      sensor_event_handler(14);
	}
	if (EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
	  /* Clear the EXTI Line 15 */
	  EXTI_ClearITPendingBit(EXTI_Line15);
	  /* UP function */
      sensor_event_handler(15);
	}

    //MainAlarmWithDuration(1);

}
  

  
/**
* @brief  This function handles external interrupts generated by UserButton.
* @param  None
* @retval None
*/


void EXTI0_IRQHandler(void)
{
  /* set UserButton Flag */
  
  EXTI_ClearITPendingBit(EXTI_Line0);

  sensor_event_handler(DI_SENSOR1);
  
  //MainAlarmWithDuration(1);
}

	
  /**
  * @brief	This function handles external interrupts generated by UserButton.
  * @param	None
  * @retval None
  */
  
  void EXTI1_IRQHandler(void)
  {
	/* set UserButton Flag */
	
	EXTI_ClearITPendingBit(EXTI_Line1);

    sensor_event_handler(DI_SENSOR2);
	
	//MainAlarmWithDuration(1);
  }

	
  /**
  * @brief	This function handles external interrupts generated by UserButton.
  * @param	None
  * @retval None
  */
  
  void EXTI2_IRQHandler(void)
  {
	/* set UserButton Flag */
	
	EXTI_ClearITPendingBit(EXTI_Line2);

	
    sensor_event_handler(DI_SENSOR3);
	//MainAlarmWithDuration(1);
  }

	
  /**
  * @brief	This function handles external interrupts generated by UserButton.
  * @param	None
  * @retval None
  */
  
  void EXTI3_IRQHandler(void)
  {
	/* set UserButton Flag */
	
	EXTI_ClearITPendingBit(EXTI_Line3);

    sensor_event_handler(DI_SENSOR4);
    
	//MainAlarmWithDuration(1);
  }

	
  /**
  * @brief	This function handles external interrupts generated by UserButton.
  * @param	None
  * @retval None
  */
  
  void EXTI4_IRQHandler(void)
  {
	/* set UserButton Flag */
	
	EXTI_ClearITPendingBit(EXTI_Line4);

	
    sensor_event_handler(DI_SENSOR5);
	//MainAlarmWithDuration(1);
  }

  /*******************************************************************************
  * Function Name  : I2C1_EV_IRQHandler
  * Description    : This function handles I2C1 Event interrupt request.
  * Input          : None
  * Output         : None
  * Return         : None
  *******************************************************************************/
  void I2C1_EV_IRQHandler(void)
  {

  }

  void hard_fault_handler_c(unsigned int * hardfault_args)   
   {   
   
       unsigned int stacked_r0;   
   
       unsigned int stacked_r1;   
   
       unsigned int stacked_r2;   
   
       unsigned int stacked_r3;   
   
       unsigned int stacked_r12;   
   
       unsigned int stacked_lr;   
   
       unsigned int stacked_pc;   
   
       unsigned int stacked_psr;   
   
         
   
       stacked_r0 = ((unsigned long) hardfault_args[0]);   
   
       stacked_r1 = ((unsigned long) hardfault_args[1]);   
   
       stacked_r2 = ((unsigned long) hardfault_args[2]);   
   
       stacked_r3 = ((unsigned long) hardfault_args[3]);   
   
         
   
       stacked_r12 = ((unsigned long) hardfault_args[4]);   
   
       stacked_lr = ((unsigned long) hardfault_args[5]);   
   
       stacked_pc = ((unsigned long) hardfault_args[6]);   
   
       stacked_psr = ((unsigned long) hardfault_args[7]);   
   
   
       printf ("[Hard fault handler]\n");   
   
       printf ("R0 = %x\n", stacked_r0);   
   
       printf ("R1 = %x\n", stacked_r1);   
   
       printf ("R2 = %x\n", stacked_r2);   
   
       printf ("R3 = %x\n", stacked_r3);   
   
       printf ("R12 = %x\n", stacked_r12);   
   
       printf ("LR = %x\n", stacked_lr);   
   
       printf ("PC = %x\n", stacked_pc);   
   
       printf ("PSR = %x\n", stacked_psr);   
   
       printf ("BFAR = %x\n", (*((volatile unsigned long *)(0xE000ED38))));   
   
       printf ("CFSR = %x\n", (*((volatile unsigned long *)(0xE000ED28))));   
   
       printf ("HFSR = %x\n", (*((volatile unsigned long *)(0xE000ED2C))));   
   
       printf ("DFSR = %x\n", (*((volatile unsigned long *)(0xE000ED30))));   
   
       printf ("AFSR = %x\n", (*((volatile unsigned long *)(0xE000ED3C))));   
   
       while(1) 
   
       { 
   
               ;; 
   
       } 
   
   
   }


  // console_write("\n\r SDIO_IRQHandler "); 
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
