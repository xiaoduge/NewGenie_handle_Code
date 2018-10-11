#ifndef _APP_AT_H_
#define _APP_AT_H_

extern uint8 appAtCmd(uint8 *pCmd,uint8 *pRsp,uint8 CmdLen,uint8 *pucRspLen);
extern void att_helper_trim(char **pstr);
extern int Att_value_parse(char *parg,char sep,char **pptag,char **ppvalue);
extern int simple_strtoul(const char *cp);
extern void Att_value_split(char *parg,char sep);

#endif
