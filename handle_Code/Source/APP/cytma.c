/*! @file radio.c
 * @brief This file contains functions to interface with the radio chip.
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2012 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <app_cfg.h>

#include "Dica.h"

#include "task.h"

#include "config.h"

#include "Bsp.h"

#include "ctype.h"

#include "beep.h"

#include "UartCmd.h"

#include "cytma.h"

#include "touch.h"

#include "sys_time.h "

#include "Relay.h"

#if (IWDG_SUPPORT > 0)
#include "IWDG_Driver.h"
#endif


#if CYTMP_SUPPORT > 0


#define TOUCH_REPORT_SIZE           10
#define TOUCH_INPUT_HEADER_SIZE     7
#define TOUCH_COUNT_BYTE_OFFSET     5
#define BTN_REPORT_SIZE             9
#define BTN_INPUT_HEADER_SIZE       5
#define SENSOR_REPORT_SIZE          150
#define SENSOR_HEADER_SIZE          4

enum ts_event_enum
{
   event_cord = 0,
   event_btn,
};

struct ts_event {
    u16 x;
    u16 y;
    u16 event;
};

struct cytma {
    
    sys_timeo           to4check; // timer for repeat check
    
    int  (*get_pendown_state)(void);
    void (*clear_penirq)(void);      

};

typedef struct
{
    MsgHead msgHead;
    void *proc;
    void *para;
}CYTMA_MSG;


#define CYTMA_MSG_LENGHT (sizeof(CYTMA_MSG)-sizeof(MsgHead))

typedef void (*cytma_msg_cb)(void *);


static struct cytma sCytma;

static void cytma_data_proc(struct cytma *ts);
	
#ifdef CYTMA_DBG_INFO
#define dev_dbg(fmt,arg...) printf(fmt,## arg)
#else
#define dev_dbg(fmt,arg...)
#endif


#define I2C_SCL_H()		(GPIOB->BSRR =  GPIO_Pin_6)
#define I2C_SCL_L()		(GPIOB->BRR =  GPIO_Pin_6)
#define I2C_SDA_H()		(GPIOB->BSRR =  GPIO_Pin_7)
#define I2C_SDA_L()		(GPIOB->BRR =  GPIO_Pin_7)
#define I2C_SDA_IS_H	((GPIOB->IDR & GPIO_Pin_7)!=0)
#define I2C_SDA_IS_L	((GPIOB->IDR & GPIO_Pin_7)==0)


//IO????	 
#define FT_RESET_H() 			(GPIOA->BSRR =  GPIO_Pin_12)		//FT5206????
#define FT_RESET_L() 			(GPIOA->BRR =  GPIO_Pin_12)			//FT5206????
#define FT_INT    				//((GPIOC->IDR & GPIO_Pin_7)!=0)		//FT5206????

//I2C????	
#define FT_CMD_WR 				0X48    	//???
#define FT_CMD_RD 				0X49		//???
  
//CYTMA568 ??????? 
#define FT_DEVIDE_MODE 			0x00   		//FT5206???????
#define FT_REG_NUM_FINGER       0x03		//???????

#define FT_TP1_REG 				0X03	  	//??????????
#define FT_TP2_REG 				0X09		//??????????
#define FT_TP3_REG 				0X0F		//??????????
#define FT_TP4_REG 				0X15		//??????????
#define FT_TP5_REG 				0X1B		//??????????  
 

#define	FT_ID_G_LIB_VERSION		0xA1		//??		
#define FT_ID_G_MODE 			0xA4   		//FT5206?????????
#define FT_ID_G_THGROUP			0x80   		//??????????
#define FT_ID_G_PERIODACTIVE	0x88   		//???????????

#define TP_NUM_MAX				(5)			// ??5???

#define TP_DEAD_LOCK_CHECK_PERIOD (10)

#define swab16(x) ((u16)(               \
    (((u16)(x) & (u16)0x00ffU) << 8) |          \
    (((u16)(x) & (u16)0xff00U) >> 8)))


#define CYTMA_NIRQ_PIN (STM32F103_GPA(0)) 

#define CYTMA_TIMER_CHECK_PERIOD (50)

#define CYTMA_IRQn EXTI0_IRQn

// add for shzn

const u16 CYTMA568_TPX_TBL[5]={FT_TP1_REG,FT_TP2_REG,FT_TP3_REG,FT_TP4_REG,FT_TP5_REG};

// 触点坐标及触点个数
s32 TpX[TP_NUM_MAX];
s32 TpY[TP_NUM_MAX];
s16 TpNum = 0;
u8 TpMask ;


u16 BtnStatus;
u8 BtnMask ;

u16 wdtTimer;

static int cymt_second;

void I2C_Wait(void);



#define DelayMs(ms) SysTick_DelayMs(ms)
#define DelayUs(us) SysTick_DelayUs(us)


/*****************************************************************

	函数功能：	初始化I2C
	输入：		无
	返回：		无
	更新日期:	2015-7-18
	备注：		无

******************************************************************/
void I2C_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	// 默认为高
	I2C_SCL_H();
	I2C_SDA_H();
	// PB6 -> SCL PB7 -> SDA
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/*************************************************
Function: I2C_Wait
Description: delay
Input: no
Output: no
More:no
*************************************************/
void I2C_Wait(void)    
{
	DelayUs(3);
}

/*************************************************
Function: I2C_Start
Description: signal of start
Input: no
Output: no
More:no
*************************************************/
void I2C_Start(void)   
{
	I2C_SDA_H();
	I2C_SCL_H();
	I2C_Wait();
	I2C_SDA_L();
	I2C_Wait();
	I2C_SCL_L();
}

/*************************************************
Function: I2C_Stop
Description: signal of stop
Input: no
Output: no
More:no
*************************************************/
void I2C_Stop(void)    
{
	I2C_SDA_L();
	I2C_Wait();
	I2C_SCL_H();
	I2C_Wait();
	I2C_SDA_H();
	I2C_Wait();
}

/*************************************************
Function: I2C_SendAck
Description: send ack to slave
Input: signal of ack
Output: no
More:no
*************************************************/
void I2C_SendAck(unsigned char ack) 
{
	
	if(ack)
	{
		I2C_SDA_H();
	}
	else
	{
		I2C_SDA_L();
	}
	I2C_Wait();
	I2C_SCL_H();
	I2C_Wait();
	I2C_SCL_L();
	I2C_Wait();
	I2C_SDA_H();
	I2C_Wait();
}

/*************************************************
Function: I2C_SendByte
Description: send data  to I2C register
Input: unsigned char of data
Output: return ack  0:receive ack  1:no ack
More:no
*************************************************/
unsigned char I2C_SendByte(unsigned char bytedata) 
{
  
	unsigned char i;
	unsigned char ack;
	I2C_Wait();
	for(i=0;i<8;i++)
	{
		if(bytedata & 0x80)
		  	I2C_SDA_H();
		else
		  	I2C_SDA_L();

		bytedata <<= 1;
		I2C_Wait();
		I2C_SCL_H();
		I2C_Wait();
		I2C_SCL_L();
		I2C_Wait();
	}

	I2C_SDA_H();
	I2C_Wait();
	I2C_SCL_H();
	I2C_Wait();
	
	ack = I2C_SDA_IS_H;    

	I2C_SCL_L();
	I2C_Wait();

	return ack;
}


/******************************************************
Function: I2C_ReceiveByte
Description: receive data  from I2C register
Input: no
Output: data 
More:no
********************************************************/
unsigned char I2C_ReceiveByte(void)  
{
	unsigned char i;
	unsigned char bytedata = 0;
	I2C_Wait();
	for(i=0;i<8;i++)
	{
		I2C_SCL_H();
		I2C_Wait();

		bytedata <<= 1;

		if(I2C_SDA_IS_H)
		{
			bytedata |= 0x01;
		}
		I2C_SCL_L();
		I2C_Wait();
	}

	return bytedata;
}

/******************************************************
Function: I2C_ByteWrite
Description: write the data to the address 
Input: device  0xC0  write to  OV6620
               0XC1  read from OV6620
               0x42  write to  OV7620
               0x43  read from OV7620
Output: no
More:no
********************************************************/

int  I2C_ByteWrite(unsigned char device,unsigned char address,unsigned char bytedata)
{     
	unsigned char ack;
	if(!I2C_SDA_IS_H) return -1;
	I2C_Start();
	ack = I2C_SendByte(device);
	if(ack)
	{
		I2C_Stop();
		return -2;
	}
	 
	ack = I2C_SendByte(address);
	if(ack)
	{
		I2C_Stop();
		return -3;
	}
	 
	ack = I2C_SendByte(bytedata);
	I2C_Stop();
	if(ack)
	{
		return -4;
	}
	return 0;
	 
}

/*****************************************************************

	函数功能：	向CYTMA568的某个起始地址写入len个字节
	输入：		-
	返回：		
				0: 成功
				非0: 失败
	更新日期:	2015-7-18
	备注：		无

******************************************************************/
u8 CYTMA568_WR_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i;
	u8 return_val=0;
	I2C_Start();	 
	I2C_SendByte(FT_CMD_WR); 	 										  		   
	I2C_SendByte(reg&0XFF); 
	I2C_SendByte(reg>>8); 
	for(i=0;i<len;i++)
	{	   
    	return_val=I2C_SendByte(buf[i]);
		if(return_val) break;  
	}
    I2C_Stop();				
	return return_val; 
}

/*****************************************************************

	函数功能：	从CYTMA568的某个起始地址读取len个字节
	输入：		-
	返回：		无
	更新日期:	2015-7-18
	备注：		无

******************************************************************/
void CYTMA568_RD_Reg(u16 reg,u8 *buf,u8 len)
{
	s8 i; 
 	I2C_Start();	
 	I2C_SendByte(FT_CMD_WR); 	 										  		   
 	I2C_SendByte(reg&0XFF);
	I2C_SendByte(reg>>8); 
 	I2C_Start();  	 	   
	I2C_SendByte(FT_CMD_RD);   
	for(i=0;i<len-1;i++)
	{	   
    	buf[i]=I2C_ReceiveByte();
		I2C_SendAck(0);
	} 
	buf[i]=I2C_ReceiveByte();
	I2C_SendAck(1);
    I2C_Stop();

} 

void cytma_hw_reset()
{
	FT_RESET_L();			
	DelayMs(50);
 	FT_RESET_H();			    
	DelayMs(50);  
}


static int cytma_get_pendown_state(void)
{
    return !stm32_gpio_get_value(CYTMA_NIRQ_PIN);
}

/*****************************************************************

	函数功能：	CYTMA568初始化
	输入：		无
	返回：		
				0: 成功
				非0: 失败
	更新日期:	2015-7-18
	备注：		无

******************************************************************/ 
u8 CYTMA568_Init(void)
{
	u8 temp[2]; 
//	u8 addr,i;
	GPIO_InitTypeDef  GPIO_InitStructure;	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	// 	PB5 -> RESET
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	TpNum=0;
	I2C_Configuration();  

    cytma_hw_reset();
    
	temp[0]=0x30;
	temp[1]=0x03;

    // 2019/05/31 add for robust workaround
    wdtTimer = 0;
	
//	addr = 0;
//	i = 1;
//	while(i)
//	{
//		I2C_Start();	 
//		i = I2C_SendByte(addr);
//		I2C_Stop();	
//		addr += 2;
//	}
//	while(1);
	
//	temp[0]=0;
//	CYTMA568_WR_Reg(FT_DEVIDE_MODE,temp,1);//进入正常操作模式 
//	CYTMA568_WR_Reg(FT_ID_G_MODE,temp,1);//查询模式 		
//	temp[0]=22;	//触摸有效值，22，越小越灵敏								
//	CYTMA568_WR_Reg(FT_ID_G_THGROUP,temp,1);//设置触摸有效值
//	temp[0]=12;	//激活周期，不能小于12，最大14							
//	CYTMA568_WR_Reg(FT_ID_G_PERIODACTIVE,temp,1); 
//	//读取版本号，参考值：0x3003
//	CYTMA568_RD_Reg(FT_ID_G_LIB_VERSION,temp,2);  
	//读取版本号，参考值：0x3003 
	if(temp[0]==0X30&&temp[1]==0X03)//版本:0X3003
	{
		return 0;
	} 
	return 1;
}

/*****************************************************************

	函数功能：	扫描触摸屏(采用查询方式)
	输入：		无
	返回：		触摸点的个数
	更新日期:	2015-7-18
	备注：		无

******************************************************************/ 
u8 CYTMA568_Scan(void)
{
	u8 buf[50];
	u8 i=0;
	u8 mode;
	u8 num=0;

#if (IWDG_SUPPORT > 0)
    IWDG_Feed();
#endif
    
    /* ylf : read input report register */
	CYTMA568_RD_Reg(FT_REG_NUM_FINGER,buf,0x11);

    switch(buf[2])
    {
    case 0x1: /* touch report */
    	mode = buf[0];
    	if(mode >= 0x11)
    	{
    		mode = (mode - TOUCH_INPUT_HEADER_SIZE) / TOUCH_REPORT_SIZE; /* ylf: record length = 10bytes ,7bytes for report header length */
    	}
    	else
    	{
            TpMask = 0;    	
    		TpNum = 0;
    		return 0x1;
    	}
    	num=mode&0XF;
    	if((num)&&(num<=TP_NUM_MAX))
    	{
    		for(i=0;i<num;i++)
    		{
    			TpX[i]=((u16)(buf[10])<<8)+buf[9];
    			TpY[i]=((u16)(buf[12])<<8)+buf[11];
	
    		} 
    		if((TpX[0]==0) && (TpY[0]==0)) 	//读到的数据都是0,则忽略此次数据
    		{
    			num=0;
    		}
    	}
    	TpNum = num;
        TpMask = (buf[8] & (3 << 5)) >> 5;
#if 0        
    	if(TpNum > 1)
    	{
    		FT_RESET_L();			
    		DelayMs(5);
    		FT_RESET_H();
    		DelayMs(5);
    	}
#endif        
    	return 0x1;        
    case 0x3: /* button report */
        BtnMask = buf[5] & 0XF;
        BtnStatus = (buf[7]<<8)+buf[6];

        //printf("btn %x ,%x\r\n",BtnMask,BtnStatus);
        return 0X3;
    default:
        return 0x0;
    }

}

void cytma_wdt_work(void)
{
   u8 cmd[32];
   int cmd_offset = 0;
   int i;
   u8 return_val=0;

   cmd[cmd_offset++] = 4;
   cmd[cmd_offset++] = 0;
   cmd[cmd_offset++] = 5;
   cmd[cmd_offset++] = 0;
   cmd[cmd_offset++] = 0x2f;
   cmd[cmd_offset++] = 0x0; /* reserved */
   cmd[cmd_offset++] = 0;

   /* Set Data */
   I2C_Start();     
   I2C_SendByte(FT_CMD_WR);                                                       
   for(i=0;i<cmd_offset;i++)
   {      
       return_val = I2C_SendByte(cmd[i]);
       if(return_val) break;  
   }
   I2C_Stop();  

   if (return_val)
   {
       cytma_hw_reset();
   }

}

void CYTMA_second(void)
{
   cymt_second++;

   if (cytma_get_pendown_state()) 
   {
       wdtTimer++;
       
       if (wdtTimer >= TP_DEAD_LOCK_CHECK_PERIOD)
       {
          VOS_LOG(VOS_LOG_ERROR,"Reset Inactive CTS \r\n");
       
          cytma_hw_reset();
       }
   }
   else
   {
        wdtTimer = 0;
        /* handshake to chip */

        if (0 == (cymt_second % 8))
        {
            cytma_wdt_work();
        }
   }

}

static void cytma_read_values(struct cytma *tsc, struct ts_event *tc)
{
   u8 num = CYTMA568_Scan();

   switch (num)
   {
   case 0x1:
       tc->x = TpY[0];
       tc->y = TpX[0];
       tc->event = event_cord;

       break;
   case 0X3:
       tc->x = BtnMask;
       tc->y = BtnMask;
       tc->event = event_btn;
       break;
   }

}


static int cytma_is_pen_down(struct cytma *ts)
{
    /*
     * NOTE: We can't rely on the pressure to determine the pen down
     * state, even though this controller has a pressure sensor.
     * The pressure value can fluctuate for quite a while after
     * lifting the pen and in some cases may not even settle at the
     * expected value.
     *
     * The only safe way to check for the pen up condition is in the
     * work function by reading the pen signal state (it's a GPIO
     * and IRQ). Unfortunately such callback is not always available,
     * in that case we assume that the pen is down and expect caller
     * to fall back on the pressure reading.
     */

    if (!ts->get_pendown_state)
        return 1;

    return ts->get_pendown_state();
}

static int cytma_open(struct cytma *ts)
{
    return 0;
}

#if 0
void cytma_report(void *proc ,void *para)
{
   Message *Msg;
   Msg = MessageAlloc(PID_CYTMA,CYTMA_MSG_LENGHT);

   if (Msg)
   {
       CYTMA_MSG *dmsg = (CYTMA_MSG *)Msg;
       dmsg->msgHead.nMsgType = CYTMA_MESSAGE_DELAY_CHECK;
       dmsg->proc = proc;
       dmsg->para = para;
       MessageSend(Msg);
   }
}

static void cytma_delay_check_proc(void *para)
{
    struct cytma *ts = (struct cytma *)para;

    cytma_data_proc(ts);
}

void cytma_delay_check_cb(void *para)
{
     cytma_report(cytma_delay_check_proc,para);
}
#endif

static void cytma_data_proc(struct cytma *ts)
{
    struct ts_event tc;
    TOUCH_EVENT te;

    wdtTimer = 0;

    //printf("tsc0 \r\n");
    //if (cytma_is_pen_down(ts)) 
    {
        /* pen is down, continue with the measurement */
        cytma_read_values(ts, &tc);

        if (event_cord == tc.event)
        {
            if (0 != TpNum)
            {
                te.usX = tc.x;
                te.usY = tc.y;
                te.usEvent = 0x1;
            }
            else
            {
                te.usX = tc.x;
                te.usY = tc.y;
                te.usEvent = 0;
            }
            touch_report(&te);
        }
        else if (event_btn == tc.event)
        {
            te.usX = tc.x;
            te.usY = tc.y;
            te.usEvent = 0x2;
            touch_report(&te);
        
        }
        // add timer
        //sys_timeout(CYTMA_TIMER_CHECK_PERIOD,SYS_TIMER_ONE_SHOT,CYTMA_TIMER_CHECK_PERIOD,cytma_delay_check_cb,ts,&ts->to4check);
        
    }
#if 0    
    else
    {
        te.usX = 0;
        te.usY = 0;
        te.usEvent = 0;
        touch_report(&te);

        sys_untimeout(&ts->to4check);
        
    }
#endif    
}

static void cytma_soft_irq(void)
{
    struct cytma *ts = &sCytma;

    cytma_data_proc(ts);

    if (ts->clear_penirq)
        ts->clear_penirq();
}

static CYTMA_msg_Handler(Message *pMsg)
{
    CYTMA_MSG *dmsg = (CYTMA_MSG *)pMsg;

    if (dmsg->proc)
    {
        ((cytma_msg_cb)dmsg->proc)(dmsg->para);
    }
}


UINT8 CYTMA_ItfProcess(Message *pMsg)
{
    switch(pMsg->msgHead.nMsgType)
    {
    case CYTMA_MESSAGE_IRQ:
        {
           // do something here
           cymt_second = 0;
           NVIC_DisableIRQ(CYTMA_IRQn);
           cytma_soft_irq();
           NVIC_EnableIRQ(CYTMA_IRQn);
        }
        break;
    case CYTMA_MESSAGE_DELAY_CHECK:
        {
           // do something here
           CYTMA_msg_Handler(pMsg);
        }
        break;
    default:
        break;
    }

    return 0;
}


int CYTMA_sh(int event,int chl,void *para)
{
    Message *Msg;
    MainAlarmWithDuration(1);

    //if (DICA_SENSOR_EVENT_FALLING == event)
    {
        Msg = MessageAlloc(PID_CYTMA,0);
        
        if (Msg)
        {
            Msg->msgHead.nMsgType = CYTMA_MESSAGE_IRQ;
            Msg->msgHead.AddData = event;
            MessageSend(Msg);
        }
    }
    return 0;
}


/*!
 *  Radio Initialization.
 *
 *  @author Sz. Papp
 *
 *  @note
 *
 */
void CYTMA_Init(void)
{
    struct cytma *ts = &sCytma;

    CYTMA568_Init();

    cytma_open(ts);

    ts->get_pendown_state = cytma_get_pendown_state;
    ts->clear_penirq      = NULL;

    stm32_gpio_cfgpin(CYTMA_NIRQ_PIN,MAKE_PIN_CFG(0,GPIO_Mode_IN_FLOATING));
    
    stm32_gpio_cfg_irq(CYTMA_NIRQ_PIN,EXTI_Trigger_Falling);
    
    // install IRQ Handler
    // InstallSensorHandler(DICA_SENSOR_EVENT_RISING,stm32_gpio_get_ext_line(CYTMA_NIRQ_PIN),FALSE,DICA_TYPE_PERIOD,CYTMA_sh,NULL);
    
    InstallSensorHandler(DICA_SENSOR_EVENT_FALLING,stm32_gpio_get_ext_line(CYTMA_NIRQ_PIN),TRUE,DICA_TYPE_PERIOD,CYTMA_sh,NULL);
    
}
#endif
