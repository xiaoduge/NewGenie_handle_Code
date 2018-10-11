#ifndef _MCP251x_DRIVER_H_
#define _MCP251x_DRIVER_H_

#include "CanDriverItf.h"


/*********************************************************************
 * Function:        BOOL CANSendMsg(  UINT16 Identifier, 
 *											UINT8* Msg, UINT8 MsgSize )
 *
 * PreCondition:    CAN initialized
 *
 * Input:			Channel: SPI channel number, 1 based
 *					Identifier: 11bit data for identifier
 *					Msg: Data bytes, 8 bytes max
 *					MsgSize: number of data bytes
 *
 * Output:          Return true if the message was successfuly transferred
 *					to the CAN controller Tx buffer.
 *
 * Side Effects:    None
 *
 * Overview:		Application call this function to send a message to the CAN bus
 *
 * Note:            None.
 ********************************************************************/
BOOL Mcp251x_CANSendMsg(CAN_FRAME *frame );


UINT8 Mcp251x_CANSendMsgNoWait(CAN_FRAME *frame );

void Mcp251x_CANGetRcvBufMsg( CAN_FRAME *frame ,UINT8 ucRxBufIndex) ;

#endif
