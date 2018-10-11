#ifndef _RTC_DRIVER_H_
#define _RTC_DRIVER_H_
typedef struct
{
       u8 hour;
       u8 min;
       u8 sec;                 
       u16 w_year;
       u8  w_month;
       u8  w_date;
       u8  week;            
}tm;

extern tm timer;


u8 RTC_Get(void);
u8 RTC_Set(u16 syear,u8 smon,u8 sday,u8 hour,u8 min,u8 sec);
void RTC_Config(void);
u8 RTC_Get_Week(u16 year,u8 month,u8 day);
u32 RTC_Get_Second(void);
int RTC_ReadBackup(u8 *data,int offset,int len);
int RTC_WriteBackup(u8 *data,int offset,int len);
void RTC_Get_Helper(u32 timecount,tm *ptm);
void RTC_Calibrate(u8 ucValue);
u32 RTC_Calc_Second(u16 syear,u8 smon,u8 sday,u8 hour,u8 min,u8 sec);

#endif
