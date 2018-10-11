#ifndef _STM32_BUTTON_H_
#define _STM32_BUTTON_H_

#define BUTTON_MAX_NUMBER (6) //  Modify according to system requirement

typedef void (*button_click_callback)(void *);
typedef void (*button_long_press_callback)(void *);


typedef enum
{
    BUTTON_TYPE_TEXT = 0,
    BUTTON_TYPE_BITMAP,
    BUTTON_TYPE_NUM,
}BUTTON_TYPE_ENUM;

typedef struct
{
    u16 usLeft;
    u16 usTop;
    u16 usHeight;
    u16 usWidth;
    u8  ucBtnType;             /* refer BUTTON_TYPE_ENUM */
    char *szName;
    const u8 *bmpNormal;
    const u8 *bmpActive;
    button_click_callback cb4c;
    button_click_callback cb4lc;

    void *tag;   /* for user para */

}BUTTON;

void BtnInit(void);
void BtnTouchEventEntry(TOUCH_EVENT *event);
BUTTON *BtnCreateButton(BUTTON *pBtn);
void BtnDestoryButton(void * btn);
void BtnShow(BUTTON *btn,BOOL bShow);

#endif


