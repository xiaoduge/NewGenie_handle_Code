#ifndef _KEY_BOARD_H_
#define _KEY_BOARD_H_
/**
  ******************************************************************************
  * @file    keyboard.h 
  * @author  STM32 Team
  * @version V1.0.0
  * @date    10/10/2014
  * @brief   Interface for keyboard access.
  ******************************************************************************
  *          DO NOT MODIFY ANY OF THE FOLLOWING CODE EXCEPT UNDER PERMISSION OF ITS PROPRIETOR
  * @copy
  *
  *
  * <h2><center>&copy; COPYRIGHT 2012 Shanghai ZhenNeng Corporation</center></h2>
  */ 


#define KEYBOARD_MAX_KEYS (8)

enum
{
  KEYSTATE_UNPRESS = 0,
  KEYSTATE_PRESSED,
  KEYSTATE_NUM,
};


typedef struct
{
    unsigned char gpio;
    unsigned char key;
}GPIO_KEY;

int RegisterKey(GPIO_KEY *key);

void  UnregisterKey(int gpio);

unsigned char keyboard_get_keystate(unsigned char key);
unsigned char keyboard_get_linestate(void);

void KeyboardInit(void);


#endif
