#include "stm32_eval.h"
#include "sys_time.h"
#include "task.h"
#include <string.h>
#include "touch.h"

void touch_report(TOUCH_EVENT *event)
{
   Message *Msg;
   Msg = MessageAlloc(PID_TOUCH,sizeof(TOUCH_EVENT));

   if (Msg)
   {
       memcpy(Msg->data,event,sizeof(TOUCH_EVENT));
       MessageSend(Msg);
   }
}

