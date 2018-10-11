#ifndef _GPIO_H_
#define _GPIO_H_

/**
  ******************************************************************************
  * @file    gpio.h 
  * @author  STM32 Team
  * @version V1.0.0
  * @date    10/10/2014
  * @brief   Interface for GPIO & Ext interrupt access.
  *          This file provides for GPIO handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  *          DO NOT MODIFY ANY OF THE FOLLOWING CODE EXCEPT UNDER PERMISSION OF ITS PROPRIETOR
  * @copy
  *
  *
  * <h2><center>&copy; COPYRIGHT 2012 Shanghai ZhenNeng Corporation</center></h2>
  */ 
#define CONFIG_STM32F103_GPIO_SPACE 0
 
 
 /* GPIO bank sizes */
#define STM32F103_GPIO_A_NR	(16)
#define STM32F103_GPIO_B_NR	(16)
#define STM32F103_GPIO_C_NR	(16)
#define STM32F103_GPIO_D_NR	(16)
#define STM32F103_GPIO_E_NR	(16)
#define STM32F103_GPIO_F_NR	(16)
#define STM32F103_GPIO_G_NR	(16)

#define STM32F103_GPIO_MASK (0xFUL)

#define STM32F103_GPIO_SHIFT (4)
 
 
#define STM32F103_GPIO_NEXT(__gpio) \
     ((__gpio##_START) + (__gpio##_NR) + CONFIG_STM32F103_GPIO_SPACE)
 
 enum stm32_gpio_number {
     STM32F103_GPIO_A_START = 0,
     STM32F103_GPIO_B_START = STM32F103_GPIO_NEXT(STM32F103_GPIO_A),
     STM32F103_GPIO_C_START = STM32F103_GPIO_NEXT(STM32F103_GPIO_B),
     STM32F103_GPIO_D_START = STM32F103_GPIO_NEXT(STM32F103_GPIO_C),
     STM32F103_GPIO_E_START = STM32F103_GPIO_NEXT(STM32F103_GPIO_D),
     STM32F103_GPIO_F_START = STM32F103_GPIO_NEXT(STM32F103_GPIO_E),
     STM32F103_GPIO_G_START = STM32F103_GPIO_NEXT(STM32F103_GPIO_F),
 
 };
 
 /* STM32F103 GPIO number definitions. */
 
#define STM32F103_GPA(_nr)	(STM32F103_GPIO_A_START + (_nr))
#define STM32F103_GPB(_nr)	(STM32F103_GPIO_B_START + (_nr))
#define STM32F103_GPC(_nr)	(STM32F103_GPIO_C_START + (_nr))
#define STM32F103_GPD(_nr)	(STM32F103_GPIO_D_START + (_nr))
#define STM32F103_GPE(_nr)	(STM32F103_GPIO_E_START + (_nr))
#define STM32F103_GPF(_nr)	(STM32F103_GPIO_F_START + (_nr))
#define STM32F103_GPG(_nr)	(STM32F103_GPIO_G_START + (_nr))

#define STM32F103_PIN2PORT(pin)((pin & (~STM32F103_GPIO_MASK)) >> STM32F103_GPIO_SHIFT)

#define MAKE_PIN_CFG(s,m) ((s<<8)|m) // s for speed,m for mode

#define offsetof(TYPE, MEMBER) ((unsigned int) &((TYPE *)0)->MEMBER)

typedef enum
{
    DICA_SENSOR_EVENT_RISING = 0,
    DICA_SENSOR_EVENT_FALLING,
    DICA_SENSOR_EVENT_NUM,
}DICA_SENSOR_EVENT_ENUM;

typedef enum
{
    DICA_TYPE_ONE_SHOT = 0,
    DICA_TYPE_PERIOD,
    DICA_TYPE_NUM,
}DICA_TYPE_ENUM;


typedef int (*dica_event_callback)(int event,int chl,void *para);


int stm32_gpio_cfgpin(unsigned int pin, unsigned int config);
unsigned int stm32_gpio_getcfg(unsigned int pin);
int stm32_gpio_get_value(unsigned  int gpio);
void stm32_gpio_set_value(unsigned  int gpio, int value);
void stm32_gpio_cfg_irq(unsigned  int gpio, int cfg);
void stm32_gpio_disable_irq(unsigned  int gpio);
void stm32_gpio_toggle_value(unsigned  int gpio);

int stm32_gpio_get_ext_state(int sensor);
int stm32_gpio_get_ext_line(unsigned  int gpio);

void InstallSensorHandler(unsigned char event,unsigned char channel,unsigned char ucCheck,unsigned char type,dica_event_callback handle,void *para);
void UninstallSensorHandler(unsigned char channel);
void UninstallSingleSensorHandler(unsigned char channel,void *Para);
int CheckSensor(int expected,int value,int bit);

void sensor_event_handler(int sensor);

void stm32_gpio_init(void);

#endif
