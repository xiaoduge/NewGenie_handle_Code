#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>

#include "stm32_eval.h"

#include "common.h"

#include "sapp.h"
#include "at.h"

#include "config.h"

extern uint16 CanAddress;
extern uint16 gusDeviceId;
extern uint16 gusOperMode;


#define AT_CONFIG_CANID    "CANID"
#define AT_CONFIG_DEVID    "DEVID"
#define AT_CONFIG_OPMODE   "OPMODE"
#define AT_CONFIG_NAME     "NAME"
#define AT_CONFIG_PSD      "PSD"

typedef enum 
{
	AT_TYPE_INT = 0,
	AT_TYPE_STRING,

}AT_TYPE_ENUM;

typedef enum 
{
	AT_ATT_NONE = 0,
	AT_ATT_PRIV,

}AT_ATT_ENUM;


typedef union
{
    uint8 u8data;
    uint16 u16data;
    uint32 u32data;
	void   *ptr;
}AT_union;

typedef void (*at_cb)(AT_union *para);

typedef struct 
{
  uint16 id;
  void  *value;
  uint8 len;
  uint8 type;
  uint8 att;
  char *buf;
  at_cb cb;
} AT_ITEM;

static uint8 sAtt = AT_ATT_NONE;


static const AT_ITEM atItem_t[] =
{
  {
    NV_DEVID_ID, &gusDeviceId,sizeof(gusDeviceId),AT_TYPE_INT,AT_ATT_NONE,AT_CONFIG_DEVID,NULL
  },
  
  {
    STM32_NV_APP_OPER_MODE, &gusOperMode,sizeof(gusOperMode),AT_TYPE_INT,AT_ATT_NONE,AT_CONFIG_OPMODE,NULL
  },
	
  {
	STM32_NV_APP_ADMIN, &gAdmin.NAME[0],ADMIN_TEXT_LENGTH,AT_TYPE_STRING,AT_ATT_PRIV,AT_CONFIG_NAME,NULL
  },

  {
    STM32_NV_APP_ADMIN+1, &gAdmin.PSD[0],ADMIN_TEXT_LENGTH,AT_TYPE_STRING,AT_ATT_PRIV,AT_CONFIG_PSD,NULL
  },
  
#if 0  
  {
    NV_CANID_ID, &CanAddress,sizeof(CanAddress),AT_CONFIG_CANID,NULL
  },
#endif  
 
  // Last item -- DO NOT MOVE IT!
  {
    0x00, 0, NULL
  }
};


void Att_value_split(char *parg,char sep)
{
    char *pvalue = strchr(parg,sep);
    
    while (pvalue)
    {
        *pvalue = 0;

         pvalue++;

         pvalue = strchr(pvalue,sep);
    }

}

void att_helper_trim(char **pstr)
{
    char *p,*q;

	p = *pstr;

	if (!p || strlen(p) == 0)
	{
		return;
	}

	while(isspace(*p)) p++;

    for ( q = p + strlen(p)-1 ; q >= p && isspace(*q) ; q-- ) 
	{
		*q = 0;
    }

	*pstr = p;

}

int Att_value_parse(char *parg,char sep,char **pptag,char **ppvalue)
{
    *pptag = parg;

    *ppvalue = strchr(*pptag,sep);
    
    if (!*ppvalue)
    {
        return FALSE;
    }
        
    **ppvalue = 0;
    
    (*ppvalue)++;

    att_helper_trim(pptag);   

    att_helper_trim(ppvalue);   

    return TRUE;
}


uint8 appAtCmd(uint8 *pCmd,uint8 *pRsp,uint8 CmdLen,uint8 *pucRspLen)
{
    //uint8 *pRsp = &sbRxBuf[SAPP_RSP_STATUS+1];  

    //uint8 *pCmd = &sbRxBuf[RPC_POS_DAT0];
    uint8 taglen;
    char *tag;
    
    
    if (CmdLen < 2)
    {
        return 0xff;
    }

    if (pCmd[0] != 'A'
        ||pCmd[1] != 'T' )
    {
        return 0XFF;
    }

    pCmd[CmdLen] = 0; // zero terminated ,for string ops

    CmdLen -= 2;

    pCmd += 2;

    if (CmdLen == 0
       || pCmd[0] == 0XD
       || pCmd[0] == 0XA)
    {

        pRsp[0] = 'O';
        pRsp[1] = 'K';
        *pucRspLen += 2;
        return 0;
    }

 
    taglen = strlen("+CONFIG=");
    tag = "+CONFIG=";
    
    if (CmdLen >= taglen)
    {
        if (0 == strncmp(tag,(char *)pCmd,taglen))
        {
            char *cfgAtt;
            char *cfgVal;
     
            CmdLen -= taglen;
            pCmd  += taglen;
     
            if (!Att_value_parse((char *)pCmd,'=',&cfgAtt,&cfgVal))
            {
                return 0XFF; // INVALID FORMAT
            }
     
            {
              uint8  i = 0;
            
              while ( atItem_t[i].id != 0x00 )
              {
                if (0 == strcmp( cfgAtt, atItem_t[i].buf))
                {
                     AT_union atdata;
                     
                     // do someting
                     if (cfgVal[0] == '?')
                     {
                         // read data

						 if (sAtt >= atItem_t[i].att)
						 {
	                         if (AT_TYPE_INT == atItem_t[i].type)
	                         {
	                             memcpy(&atdata,atItem_t[i].value,atItem_t[i].len);
	                             *pucRspLen += sprintf((char *)pRsp,"%s:%u",atItem_t[i].buf,(atItem_t[i].len == sizeof(uint8) )? atdata.u8data : atItem_t[i].len == sizeof(uint16) ? atdata.u16data: atdata.u32data);
	                         }
							 else
							 {
	                             *pucRspLen += sprintf((char *)pRsp,"%s:%s",atItem_t[i].buf,(char *)atItem_t[i].value);
							 }
						 }
						 else
					 	 {
                             *pucRspLen += sprintf((char *)pRsp,"config %s: unauthorized",atItem_t[i].buf);
							 return 0;
					 	 }
                         
                         return 0;
                     }
                     else
                     {
                         uint8 ucStatus;
                         if (atItem_t[i].len == sizeof(uint8))
                         {
                              atdata.u8data  = atoi(cfgVal);
                         }
                         else if (atItem_t[i].len == sizeof(uint16))
                         {
                              atdata.u16data  = atoi(cfgVal);
                         }
                         else if (atItem_t[i].len == sizeof(uint32))
                         {
                              atdata.u32data  = atoi(cfgVal);
                         }

						 if (AT_TYPE_INT == atItem_t[i].type)
						 {
						     memcpy(atItem_t[i].value,&atdata,atItem_t[i].len);
						 }
						 else
					 	 {
					 	     if (strlen(cfgVal) <= atItem_t[i].len)
					 	     {
						         strcpy(atItem_t[i].value,cfgVal);
					 	     }
					 	 }

						 if (sAtt >= atItem_t[i].att)
						 {
	                         ucStatus = Config_SetItem(atItem_t[i].id,atItem_t[i].len,&atdata);
	                         
	                         if (atItem_t[i].cb)atItem_t[i].cb(&atdata);
	                         
	                         return ucStatus;
						 }
						 else
					 	 {
                             *pucRspLen += sprintf((char *)pRsp,"config %s: unauthorized",atItem_t[i].buf);
							 return 0;
					 	 }
                     }
                     
                     //break;
                }
            
                // Move on to the next item
                i++;
              }
     
            }  
            return 0XFF;
        }
    }

    taglen = strlen("+LOG=");
    tag = "+LOG=";
    
    if (CmdLen >= taglen)
    {
        if (0 == strncmp(tag,(char *)pCmd,taglen))
        {
            char *szName = "";
            char *szPass = "";
			char *value;
     
            CmdLen -= taglen;
            pCmd  += taglen;

			value = (char *)pCmd;
     
            Att_value_split((char *)value,'&');

			if (strlen(value) > 0)
			{
				szName = value;
				value += strlen(value) + 1;
			}
			
			if (strlen(value) > 0)
			{
				szPass = value;
				value += strlen(value) + 1;
			}	

			att_helper_trim(&szName);
			att_helper_trim(&szPass);
			
			if (strcmp(gAdmin.NAME, szName) == 0 
				&& strcmp(gAdmin.PSD, szPass) == 0)
			{
			    sAtt = AT_ATT_PRIV;
				*pucRspLen += sprintf((char *)pRsp,"LOG %s&%s: Succeed",szName,szPass);
				return 0;
				
			}
			else
			{
				sAtt = AT_ATT_NONE;
				*pucRspLen += sprintf((char *)pRsp,"LOG %s&%s: Failed",szName,szPass);
				return 0;
				
			}
        }
    }
    
    taglen = strlen("+DBG=");
    tag = "+DBG=";
    
    if (CmdLen >= taglen)
    {
        if (0 == strncmp(tag,(char *)pCmd,taglen))
        {
            void zb_PrepareReset(void);
        
            CmdLen -= taglen;
            pCmd  += taglen;

            zb_PrepareReset();					
           
		    return 0;
        }
    }

    return 0xff;
    
}


