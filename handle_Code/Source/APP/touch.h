#ifndef _TOUCH_H_
#define _TOUCH_H_

typedef struct
{
    uint16_t usX;
    uint16_t usY;
    uint16_t usEvent;
}TOUCH_EVENT;

void touch_report(TOUCH_EVENT *event);
#endif
