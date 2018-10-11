#ifndef _STORAGE_H_
#define _STORAGE_H_



uint8_t Storage_ReadRunTimeInfo(DISPLAY_STRU *pDisp);

uint8_t Storage_WriteRunTimeInfo(DISPLAY_STRU *pDisp);

uint8_t Storage_WriteMeasInfo(DISPLAY_STRU *pDisp,tm *ptm);

uint8_t Storage_WriteRunTimeInfo2Flash(DISPLAY_STRU *pDisp);

void Storage_Init(void);

int Storage_AddUser(char *szName,char *szPass,char attr);

int Storage_RmvUser(char *szName);

int Storage_UpdateUser(STORAGE_USER_INFO *pUsrInfo);

int Storage_CheckUser(char *szName,char *szPass);

int Storage_GetUser(char *szName,STORAGE_USER_INFO *ui);

int Storage_GetMeasInfo(uint8 *iData,tm *ptm);


#endif
