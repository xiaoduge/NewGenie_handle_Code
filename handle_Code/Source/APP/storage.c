#include <string.h>
#include "Display.h"

#include "RTC_Driver.h"

#include "SDDriver_hal.h"

#include "storage.h"

#include "Config.h"

#include "UartCmd.h"

#include "ff.h"

#include <string.h>

#include <stdio.h>


uint8_t Storage_WriteMeasInfo(DISPLAY_STRU *pDisp,tm *ptm)
{
}

int Storage_GetMeasInfo(uint8 *iData,tm *ptm)
{
}

int Storage_AddUser(char *szName,char *szPass,char attr)
{
}



int Storage_RmvUser(char *szName)
{
}


int Storage_UpdateUser(STORAGE_USER_INFO *pUsrInfo)
{
}

int Storage_CheckUser(char *szName,char *szPass)
{
    
}

int Storage_GetUser(char *szName,STORAGE_USER_INFO *ui)
{
    
}

void Storage_Init(void)
{

}
