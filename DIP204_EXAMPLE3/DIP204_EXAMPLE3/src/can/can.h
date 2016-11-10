/*
 * CAN.h
 *
 *  Created on: 28 jun 2010
 *      Author: permac2
 */

#ifndef CAN_H_
#define CAN_H_

#include <avr32/io.h>
#include "compiler.h"
#include "dip204.h"

#define PRIVATE   static
#define TRUE	1
#define FALSE	0

typedef unsigned short int  UINT16;
typedef unsigned long  UINT32;
typedef unsigned char       UINT8;

extern void InitializeCAN( int Channel, int BusSpeed, UINT16 Mask, UINT16 Flt[]);
extern void InitializeCANExtended( int Channel , int BusSpeed, UINT32 Mask, UINT32 Flt[]);
extern void CANEnable( int Channel, int BusSpeed );
extern void CANReset(int Channel);
extern void CAN2515SetRXB0Filters(int Channel, UINT16 Mask0, UINT16* pFlt0_1 );
extern void CAN2515SetRXB1Filters(int Channel, UINT16 Mask1, UINT16* pFlt2_5 );
extern void CAN2515SetRXB0FiltersEx(int Channel, UINT32 Mask0, UINT32* pFlt0_1);
extern void CAN2515SetRXB1FiltersEx(int Channel, UINT32 Mask1, UINT32* pFlt2_5);
extern void config_SPI_SPARE(void);
extern void config_dpi204(void);
extern inline Bool CANTxReady(int Channel);
extern inline Bool CANRxReady(int Channel);
extern Bool CANGetMsg( int Channel, UINT32* pIdentifier, UINT8* Msg, UINT8* pMsgSize );
extern Bool CANSendMsg( int Channel, UINT32 IdentifierS, UINT8* MsgS, UINT8 MsgSizeS, int r );
extern void Evk1100PrintDisp(UINT32* pIdentifier, UINT8* Msg, UINT8* pMsgSize );
extern void ClearMessages(UINT8* Msg);
#endif /* CAN_H_ */
