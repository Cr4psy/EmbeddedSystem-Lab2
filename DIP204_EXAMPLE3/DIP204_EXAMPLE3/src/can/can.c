/*
 * CAN.c
 */
#include "can.h"
#include "spi.h"
#include "gpio.h"
#include "board.h"
#include "regs2515.h"
#include "delay.h"

#define PRIVATE   static
#define XMIT_TIMEOUT 100	//milli sec
#define RemoteFrameId 0x01
#define R 82 // Dec ASCII

/*********************************************************************
 * Function:        void ClearMessages(UINT8* Msg)
 *
 * PreCondition:    None
 *
 * Input:			Pointer to array of UINT8.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		This function clears the message content
 *
 * Note:            None.
 *
 * Example:			ClearMessages(msg);
 ********************************************************************/
void ClearMessages(UINT8* Msg)
{
	Msg[0] = 0;
	Msg[1] = 0;
	Msg[2] = 0;
	Msg[3] = 0;
	Msg[4] = 0;
	Msg[5] = 0;
	Msg[6] = 0;
	Msg[7] = 0;
}
/*********************************************************************
 * Function:        void Evk1100PrintDisp(UINT32* pIdentifier, UINT8* Msg, UINT8* pMsgSize)
 *
 * PreCondition:    void config_dpi204(void) has to be run first
 *
 * Input:			pIdentifier: Pointer to Identifier of UINT32
 *					Msg:         Pointer to array of UINT8
 *					pMsgSize: Pointer to the size of the messaged of UINT8
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		This function prints all eight message bytes, the identifier and the message size
 *
 * Note:            None.
 *
 * Example:			Evk1100PrintDisp(&Ident, msg, &mSize );
 ********************************************************************/
void Evk1100PrintDisp(UINT32* pIdentifier, UINT8* Msg, UINT8* pMsgSize )
{
    dip204_set_cursor_position(1,1);
    dip204_printf_string("%x", Msg[0]);
    dip204_set_cursor_position(6,1);
    dip204_printf_string("%x", Msg[1]);
    dip204_set_cursor_position(11,1);
    dip204_printf_string("%x", Msg[2]);
    dip204_set_cursor_position(16,1);
    dip204_printf_string("%x", Msg[3]);
    dip204_set_cursor_position(1,2);
    dip204_printf_string("%x", Msg[4]);
    dip204_set_cursor_position(6,2);
    dip204_printf_string("%x", Msg[5]);
    dip204_set_cursor_position(11,2);
    dip204_printf_string("%x", Msg[6]);
    dip204_set_cursor_position(16,2);
    dip204_printf_string("%x", Msg[7]);
    dip204_set_cursor_position(1,3);
    dip204_printf_string("Id: ");
    dip204_set_cursor_position(4,3);
    dip204_printf_string("%x", *pIdentifier);
    dip204_set_cursor_position(13,3);
    dip204_printf_string("DLC:");
    dip204_set_cursor_position(17,3);
    dip204_printf_string("%x", *pMsgSize);
}
/*********************************************************************
 * Function:        void config_SPI_SPARE(void)
 *
 * PreCondition:    None
 *
 * Input:			None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		This function setups the SPI spare port. This to be able to communicate with other devices, e.g. MCP2515
 *
 * Note:            None.
 *
 * Example: 	    config_SPI_SPARE();
 ********************************************************************/
void config_SPI_SPARE(void){
static const gpio_map_t SPARE_SPI_GPIO_MAP =
{
{SPARE_SPI_SCK_PIN,  SPARE_SPI_SCK_FUNCTION },  // SPI Clock.
{SPARE_SPI_MISO_PIN, SPARE_SPI_MISO_FUNCTION},  // MISO.
{SPARE_SPI_MOSI_PIN, SPARE_SPI_MOSI_FUNCTION},  // MOSI.
{SPARE_SPI_NPCS_PIN, SPARE_SPI_NPCS_FUNCTION}   // Chip Select NPCS.
};

spi_options_t spiOptions =
{
.reg          = SPARE_SPI_NPCS,
.baudrate     = 1000000,
.bits         = 8,
.spck_delay   = 0,
.trans_delay  = 0,
.stay_act     = 0,
.spi_mode     = 0,
.modfdis      = 1
};

gpio_enable_module(SPARE_SPI_GPIO_MAP,
sizeof(SPARE_SPI_GPIO_MAP) / sizeof(SPARE_SPI_GPIO_MAP[0]));

spi_initMaster(SPARE_SPI,&spiOptions);
spi_selectionMode(SPARE_SPI, 0, 0, 0);
spi_selectChip(SPARE_SPI,0);
spi_setupChipReg(SPARE_SPI, &spiOptions, FOSC0);
spi_enable(SPARE_SPI);
}
/*********************************************************************
 * Function:        void mASSERT_CS(unsigned char channel)
 *
 * PreCondition:    void config_SPI_SPARE(void) has to run first
 *
 * Input:			channel: channel of unsigned char
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		See the documentation for spi_selectchip in spi.h
 *
 * Note:            spi.h has to be included.
 ********************************************************************/
void mASSERT_CS(unsigned char channel){
	spi_selectChip(SPARE_SPI,channel);
}
/*********************************************************************
 * Function:        void mDEASSERT_CS(unsigned char channel)
 *
 * PreCondition:    void config_SPI_SPARE(void) has to run first
 *
 * Input:			channel: channel of unsigned char
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		See the documentation for spi_unselectchip in spi.h
 *
 * Note:            spi.h has to be included.
 ********************************************************************/
void mDEASSERT_CS(unsigned char channel){
	spi_unselectChip(SPARE_SPI,channel);
}
/*********************************************************************
 * Function:        PRIVATE UINT8 XferSPI_send( int Channel, UINT8 dat )
 *
 * PreCondition:    void config_SPI_SPARE(void) has to run first
 *
 * Input:			channel: channel of type int
 * 					dat:     data to send
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Selects a slave in master variable peripheral select mode and writes
 *         			one data word to it. See the documentation for spi_write in spi.h
 *
 * Note:            spi.h has to be included.
 ********************************************************************/
PRIVATE UINT8 XferSPI_send( int Channel, UINT8 dat )
{
	spi_write(SPARE_SPI, dat);
	return 0;
}
/*********************************************************************
 * Function:        PRIVATE UINT8 XferSPI_receive( int Channel, UINT8 dat )
 *
 * PreCondition:    void config_SPI_SPARE(void) has to run first
 *
 * Input:			channel: channel of type int
 * 					dat:     data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		See the documentation for spi_write and spi_read in spi.h
 *
 * Note:            spi.h has to be included.
 ********************************************************************/
PRIVATE UINT8 XferSPI_receive( int Channel, UINT8 dat )
{
	unsigned short *spidatareadpointer_temp;
	unsigned short spidataread_temp;
	spidatareadpointer_temp=&spidataread_temp;

	spi_write(SPARE_SPI, DUMMY_BYTE);
	while(spi_readRegisterFullCheck(SPARE_SPI)==0){
		asm("NOP;");
	}
	spi_read(SPARE_SPI,spidatareadpointer_temp);
	return spidataread_temp;
}
/*********************************************************************
 * Function:        void CANReset(int Channel)
 *
 * PreCondition:    SPI initialization and configuration
 *
 * Input:			channel: channel of type int
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Sends a software reset commmand over spi port to MCP2515 chip
 *
 * Note:            None.
 ********************************************************************/
void CANReset(int Channel)
{
	mASSERT_CS(Channel);
	XferSPI_send(Channel, CAN_RESET);
	mDEASSERT_CS(Channel);
}
/*********************************************************************
 * Function:        void CAN2515ByteWrite(int Channel, unsigned char addr, unsigned char value )
 *
 * PreCondition:    SPI initialization and configuration
 *
 * Input:			channel: channel of type int
 * 					addr:    address of type unsigned char
 * 					value:   the value to be send to the address
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		This function is used for setting register values e.g. registers in the MCP2515 module
 *
 * Note:            None.
 *
 * Example:         CANReset(0)
 ********************************************************************/
void CAN2515ByteWrite(int Channel, unsigned char addr, unsigned char value )
{
	mASSERT_CS(Channel);
	XferSPI_send(Channel, CAN_WRITE);
	XferSPI_send(Channel, addr);
	XferSPI_send(Channel, value);
	mDEASSERT_CS(Channel);
}
/*********************************************************************
 * Function:        PRIVATE UINT8 CAN2515ByteRead(int Channel, unsigned char addr)
 *
 * PreCondition:    SPI initialization
 *
 * Input:			channel: channel of type int
 * 					addr:    address of type unsigned char
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		This function reads the value of a byte. It is used to read register values in e.g. the MCP2515 module
 *
 * Note:            None.
 ********************************************************************/
PRIVATE UINT8 CAN2515ByteRead(int Channel, unsigned char addr)
{
unsigned char tempdata;

	mASSERT_CS(Channel);
	XferSPI_send(Channel, CAN_READ);
	XferSPI_send(Channel, addr);
	tempdata = XferSPI_receive(Channel,0);
	mDEASSERT_CS(Channel);

	return tempdata;
}
/*********************************************************************
 * Function:        void CAN2515SetRXB0Filters(int Channel, UINT16 Mask0, UINT16* pFlt0_1 )
 *
 * PreCondition:    SPI port configured
 *
 * Input:			Channel: SPI channel number, 1 based
 *					Mask0:	 define identifier 11 bits that are must match.
 *					pFlt0_1: Pointer to array of UINT16, 2 words,
 *							 ie. Filter 0 and Filer 1.  Lower 11 bits are siginicant
 *							 1 = Rcvd Idenifier bit must match filter bit
 *							 0 = Don't care Rcvd identifier bit.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Initialize RXB0 mask and 2 filters. See MCP2515 datasheet for more information.
 *
 * Note:            None.
 *
 * Example:			CAN2515SetRXB0Filters(Channel, 0, Flt);
 ********************************************************************/
void CAN2515SetRXB0Filters(int Channel, UINT16 Mask0, UINT16* pFlt0_1 )
{
	CAN2515ByteWrite(Channel, RXM0SIDH, Mask0 >> 3);
	CAN2515ByteWrite(Channel, RXM0SIDL, Mask0 << 5);

	// Set two filters associated with RXB0

	CAN2515ByteWrite(Channel, RXF0SIDH, *pFlt0_1 >> 3);
	CAN2515ByteWrite(Channel, RXF0SIDL, *pFlt0_1 << 5);

	pFlt0_1++;
	CAN2515ByteWrite(Channel, RXF1SIDH, *pFlt0_1 >> 3);
	CAN2515ByteWrite(Channel, RXF1SIDL, *pFlt0_1 << 5);

}
/*********************************************************************
 * Function:        void CAN2515SetRXB1Filters(int Channel,UINT16 Mask0, UINT16* pFlt0_1 )
 *
 * PreCondition:    SPI port configured
 *
 * Input:			Channel: SPI channel number, 1 based
 *					Mask0:	 define identifier 11 bits that are must match.
 *					pFlt2_5: Pointer to array of UINT16, 4 words,
 *							 ie. Filter 2 to Filter 5.  Lower 11 bits are significant.
 *							 1 = Rcvd Identifier bit must match filter bit
 *							 0 = Don't care Rcvd identifier bit.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Initialize RXB1 mask and 4 filters. See MCP2515 datasheet for more information.
 *
 * Note:            None.
 *
 * Example:			CAN2515SetRXB1Filters(Channel, 0, &Flt[2]);
 ********************************************************************/
void CAN2515SetRXB1Filters(int Channel, UINT16 Mask1, UINT16* pFlt2_5 )
{
	CAN2515ByteWrite(Channel, RXM1SIDH, Mask1 >> 3);
	CAN2515ByteWrite(Channel, RXM1SIDL, Mask1 << 5);

	// Set Four filters associated with RXB1

	CAN2515ByteWrite(Channel, RXF2SIDH, *pFlt2_5 >> 3);
	CAN2515ByteWrite(Channel, RXF2SIDL, (*pFlt2_5 << 5) |0x08 );

	pFlt2_5++;
	CAN2515ByteWrite(Channel, RXF3SIDH, *pFlt2_5 >> 3);
	CAN2515ByteWrite(Channel, RXF2SIDL, (*pFlt2_5 << 5) |0x08 );

	pFlt2_5++;
	CAN2515ByteWrite(Channel, RXF4SIDH, *pFlt2_5 >> 3);
	CAN2515ByteWrite(Channel, RXF2SIDL, (*pFlt2_5 << 5) |0x08 );

	pFlt2_5++;
	CAN2515ByteWrite(Channel, RXF5SIDH, *pFlt2_5 >> 3);
	CAN2515ByteWrite(Channel, RXF2SIDL, (*pFlt2_5 << 5) |0x08 );
}
/*********************************************************************
 * Function:        void CAN2515SetRXB0FiltersEx(int Channel, UINT32 Mask0, UINT32* pFlt0_1 )
 *
 * PreCondition:    SPI port configured
 *
 * Input:			Channel: SPI channel number, 1 based
 *					Mask0:  The Mask for the RXBO buffer.
 *							1 = Rcvd Identifier bit must match filter bit
 *							0 = Don't care about Rcvd identifier bit.
 *					pFlt0_1: Pointer to array of UINT32, 2 words, 
 *							 ie. Filter 0 and Filer 1.  Lower 29 bits are significant.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Initialize RXB0 mask and 2 filters. See MCP2515 data sheet for more information.
 *
 * Note:            None.
 *
 * Example:			CAN2515SetRXB0Filters(Channel, 0, Flt);
 ********************************************************************/
void CAN2515SetRXB0FiltersEx(int Channel, UINT32 Mask0, UINT32* pFlt0_1)
{	
	//Set the mask associated with RXB0
	CAN2515ByteWrite(Channel, RXM0EID0, Mask0);
	CAN2515ByteWrite(Channel, RXM0EID8, Mask0 >> 8);
	CAN2515ByteWrite(Channel, RXM0SIDL, ((Mask0 >> 13) & 0b11100000) | ((Mask0 >> 16) & 0b00000011));
	CAN2515ByteWrite(Channel, RXM0SIDH, Mask0 >> 21);
	    
	//Set the two filters associated with RXB0
	CAN2515ByteWrite(Channel, RXF0EID0, *pFlt0_1);
	CAN2515ByteWrite(Channel, RXF0EID8, *pFlt0_1 >> 8);
	CAN2515ByteWrite(Channel, RXF0SIDL, ((*pFlt0_1 >> 13) & 0b11100000) | 0b00001000 | ((*pFlt0_1 >> 16) & 0b00000011));
	CAN2515ByteWrite(Channel, RXF0SIDH, *pFlt0_1 >> 21);
	    
	pFlt0_1++;
	CAN2515ByteWrite(Channel, RXF1EID0, *pFlt0_1);
	CAN2515ByteWrite(Channel, RXF1EID8, *pFlt0_1 >> 8);
	CAN2515ByteWrite(Channel, RXF1SIDL, ((*pFlt0_1 >> 13) & 0b11100000) | 0b00001000 | ((*pFlt0_1 >> 16) & 0b00000011));
	CAN2515ByteWrite(Channel, RXF1SIDH, *pFlt0_1 >> 21);
}
/*********************************************************************
 * Function:        void CAN2515SetRXB0FiltersEx(int Channel, UINT32 Mask1, UINT32* pFlt2_5)
 *
 * PreCondition:    SPI port configured
 *
 * Input:			Channel: SPI channel number, 1 based
 *					Mask1:  The Mask for the RXB1 buffer.
 *							1 = Rcvd Identifier bit must match filter bit
 *							0 = Don't care about Rcvd identifier bit.
 *					pFlt2_5: Pointer to array of UINT32, 4 words, 
 *							 i.e. Filter 2 to Filter 5.  Lower 29 bits are significant.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Initialize RXB0 mask and 2 filters. See MCP2515 data sheet for more information.
 *
 * Note:            None.
 *
 * Example:			CAN2515SetRXB0Filters(Channel, 0, Flt);
 ********************************************************************/
void CAN2515SetRXB1FiltersEx(int Channel, UINT32 Mask1, UINT32* pFlt2_5)
{ 
	    //Set the mask associated with RXB1
	    CAN2515ByteWrite(Channel, RXM1EID0, Mask1);
	    CAN2515ByteWrite(Channel, RXM1EID8, Mask1 >> 8);
	    CAN2515ByteWrite(Channel, RXM1SIDL, ((Mask1 >> 13) & 0b11100000) | ((Mask1 >> 16) & 0b00000011));
	    CAN2515ByteWrite(Channel, RXM1SIDH, Mask1 >> 21);
	    
	    //Set the 4 filters associated with RXB1
	    CAN2515ByteWrite(Channel, RXF2EID0, *pFlt2_5);
	    CAN2515ByteWrite(Channel, RXF2EID8, *pFlt2_5 >> 8);
	    CAN2515ByteWrite(Channel, RXF2SIDL, ((*pFlt2_5 >> 13) & 0b11100000) | 0b00001000 | ((*pFlt2_5 >> 16) & 0b00000011));
	    CAN2515ByteWrite(Channel, RXF2SIDH, *pFlt2_5 >> 21);
	    
	    pFlt2_5++;
	    CAN2515ByteWrite(Channel, RXF3EID0, *pFlt2_5);
	    CAN2515ByteWrite(Channel, RXF3EID8, *pFlt2_5 >> 8);
	    CAN2515ByteWrite(Channel, RXF3SIDL, ((*pFlt2_5 >> 13) & 0b11100000) | 0b00001000 | ((*pFlt2_5 >> 16) & 0b00000011));
	    CAN2515ByteWrite(Channel, RXF3SIDH, *pFlt2_5 >> 21);
	    
	    pFlt2_5++;
	    CAN2515ByteWrite(Channel, RXF4EID0, *pFlt2_5);
	    CAN2515ByteWrite(Channel, RXF4EID8, *pFlt2_5 >> 8);
	    CAN2515ByteWrite(Channel, RXF4SIDL, ((*pFlt2_5 >> 13) & 0b11100000) | 0b00001000 | ((*pFlt2_5 >> 16) & 0b00000011));
	    CAN2515ByteWrite(Channel, RXF4SIDH, *pFlt2_5 >> 21);
	    
	    pFlt2_5++;
	    CAN2515ByteWrite(Channel, RXF5EID0, *pFlt2_5);
	    CAN2515ByteWrite(Channel, RXF5EID8, *pFlt2_5 >> 8);
	    CAN2515ByteWrite(Channel, RXF5SIDL, ((*pFlt2_5 >> 13) & 0b11100000) | 0b00001000 | ((*pFlt2_5 >> 16) & 0b00000011));
	    CAN2515ByteWrite(Channel, RXF5SIDH, *pFlt2_5 >> 21);
}
/*********************************************************************
 * Function:        PRIVATE UINT8 ReadStatus2515(int Channel)
 *
 * PreCondition:    SPI port configured
 *
 * Input:			Channel: SPI channel number, 1 based
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Reads status from MCP2515
 *
 * Note:            None.
 ********************************************************************/
PRIVATE UINT8 ReadStatus2515(int Channel)
{
	unsigned short *spidatareadpointer_temp;
	unsigned short spidataread_temp;
	spidatareadpointer_temp=&spidataread_temp;

	spi_selectChip(SPARE_SPI,0);

	spi_write(SPARE_SPI,CAN_RD_STATUS);

	spi_write(SPARE_SPI,DUMMY_BYTE);
	spi_read(SPARE_SPI,spidatareadpointer_temp);

	spi_unselectChip(SPARE_SPI,0);

	return spidataread_temp;
}
/*********************************************************************
 * Function:        BOOL CANSendMsg( int Channel, UINT16 Identifier,UINT8* Msg, UINT8 MsgSize, int r )
 *
 * PreCondition:    CAN initialized
 *
 * Input:			Channel: SPI channel number, 1 based
 *					Identifier: 11bit or 29 bit data for identifier
 *					Msg: Data bytes, 8 bytes max
 *					MsgSize: number of data bytes
 *					r: If user wants to send a remote frame or not
 *
 * Output:          Return true if the message was successfuly transferred
 *					to the CAN controller Tx buffer.
 *
 * Side Effects:    None
 *
 * Overview:		Application call this function to send a message to the CAN bus
 *
 * Note:            None.
 *
 * Example:			// Channel, Identifier (max 0x1fffffff (29 bits)), Message, Number of bytes, Remote frame R or 0 (no remote frame)

			        // Standard id
                    CANSendMsg( 0, 0x00, msg, 8, 0 );(no remote frame)
					CANSendMsg( 0, 0x00, msg, 8, R );(remote frame)

					// Extended id
                    CANSendMsg( 0, 0x8ff, msg, 8, 0 );(no remote frame)
				    CANSendMsg( 0, 0x8ff, msg, 8, R );(remote frame)
 ********************************************************************/
Bool CANSendMsg( int Channel, UINT32 IdentifierS, UINT8* MsgS, UINT8 MsgSizeS, int r )
{
	int WaitCntr = 0;

	// wait for TXB0 to get ready.  If not ready within XMIT_TIMEOUT ms,then return false
	while( CAN2515ByteRead( Channel, TXB0CTRL ) & 0x08 ) //TXREQ == 1
	{
		delay_ms( 1 );
		if( WaitCntr++ >= XMIT_TIMEOUT )
			return FALSE;
	}
	if ((IdentifierS>>11)==0) // Standard id
	{
		CAN2515ByteWrite(Channel, TXB0SIDH, (IdentifierS >> 3) & 0xff );    //Set TXB0 SIDH
		CAN2515ByteWrite(Channel, TXB0SIDL,((IdentifierS << 5) & 0xe0));    //Set TXB0 SIDL
	}
	else // Extended id
	{
		CAN2515ByteWrite(Channel, TXB0SIDH, (IdentifierS >> 21) & 0xff );    //Set TXB0 SIDH
		CAN2515ByteWrite(Channel, TXB0SIDL,(((IdentifierS >> 13)& 0xe0) | ((IdentifierS>>16) & 0x03) )| 0x08 ); //Set TXB0 SIDL
		CAN2515ByteWrite(Channel, TXB0EID8, (IdentifierS>>8) & 0xff);
		CAN2515ByteWrite(Channel, TXB0EID0, (IdentifierS & 0xff));
	}

	if( MsgSizeS > 8 )
		MsgSizeS = 8;

	if( r==82) //Remote frame
		{
			CAN2515ByteWrite(Channel, TXB0DLC, (MsgSizeS |=0x40));  //Set DLC
			dip204_set_cursor_position(17,4);
			dip204_printf_string("Re F");
		}
	else // No remote frame
		{
		CAN2515ByteWrite(Channel, TXB0DLC, MsgSizeS);  //Set DLC
		}

	int temp;
	for( temp = 0; temp < MsgSizeS; temp++ )
	  CAN2515ByteWrite( Channel, TXB0D0+temp, MsgS[temp] );

	CAN2515ByteWrite( Channel, TXB0CTRL, 0x08 ); //Start Transmission.

	return TRUE;
}
/*********************************************************************
 * Function:        BOOL CANGetMsg( int Channel, UINT16 *pIdentifier,UINT8* Msg, UINT8 *pMsgSize )
 *
 * PreCondition:    CAN initialized
 *
 * Input:			Channel: SPI channel number, 1 based
 *					Identifier: 11bit or 29bit data for identifier
 *					Msg: Data bytes, 8 bytes max
 *					MsgSize: number of data bytes
 *
 * Output:          Return true if a message is received
 *
 * Side Effects:    None
 *
 * Overview:		Application call this function to read the CAN message
 *					received by the CAN controller
 *
 * Note:            None.
 *
 * Example:         CANGetMsg(0, &Ident, msg, &mSize );
 ********************************************************************/
Bool CANGetMsg( int Channel, UINT32* pIdentifier, UINT8* Msg, UINT8* pMsgSize )
{
        int temp;
        UINT8 loc, S1, S2, S3, S4;

        temp = ReadStatus2515(Channel);

        if( (temp & 3) == 0 )
            return FALSE;
        temp&=0x03;
        if(temp==1){
            loc=0x61;
        }
        else if (temp==2){
            loc=0x71;
        }
        else{
            return FALSE;
        }

        S1=CAN2515ByteRead(Channel, loc);
        S2=CAN2515ByteRead(Channel, loc+1);
        S3=CAN2515ByteRead(Channel, loc+2);
        S4=CAN2515ByteRead(Channel, loc+3);

        if (((S2>>3)&0x01)==0) //format the 11 bit identifier
        {
            *pIdentifier = S1<<3 | S2>>5;
            LED_On(LED0);
            LED_Off(LED1);

             //dip204_set_cursor_position(1,4);
             //dip204_printf_string("St Id");
        }
        else if (((S2>>3)&0x01)==1) //format the 29 bit identifier
        {
            *pIdentifier =  (S1<<21 |((S2>>3 & 0x1c)|(S2&0x03))<<16 | S3 <<8 |  S4);
            LED_On(LED1);
            LED_Off(LED0);

            //dip204_set_cursor_position(1,4);
            //dip204_printf_string("Ex Id");
        }

        *pMsgSize = CAN2515ByteRead(Channel, loc+4) & 0x0F; //Data Length

        if(*pMsgSize>8)
            *pMsgSize = 8;

        for( temp = 0; temp < *pMsgSize; temp++ ){
            Msg[temp] = CAN2515ByteRead(Channel, loc+5+temp);
        }

        // Here the RXRTR bit is check to see if a remote frame was received.
        // Here is the identifier of the remote frame being set. When a remote frame messages with the same identifier as defined here the
        //node will respond with a user predefined message.
        //UINT8 RemoteFrameId=User defined;
        LED_Off(LED2);
        if ( ((CAN2515ByteRead( Channel, RXB0CTRL ) & 0x08) || (CAN2515ByteRead( Channel, RXB1CTRL ) & 0x08)) && *pIdentifier==RemoteFrameId)
                    {
                    Msg[0] = 0;
                    Msg[1] = 1;
                    Msg[2] = 2;
                    Msg[3] = 3;

                    CANSendMsg( Channel,*pIdentifier, Msg, 4, 0 );
                    dip204_set_cursor_position(17,4);
                    dip204_printf_string("Re F");
                    LED_On(LED2);
                }

        //clear CANINTF RX01F_RESET=0x00. To be able to receive new messages
        CAN2515ByteWrite(0,CANINTF,RX0IF_RESET);
        CAN2515ByteWrite(0,CANINTF,RX1IF_RESET);

    return TRUE;
}
/*********************************************************************
 * Function:        void CANEnable(int Channel, int BusSpeed )
 *
 * PreCondition:    SPI port configured, CANReset is called, and RXB
 *					Filters intitialized.
 *
 * Input:			Channel: SPI channel number, 1 based
 *					BusSpeed: Bus Speed code: CAN_1000kbps, CAN_500kbps
 *							CAN_250kbps, or CAN_125kbps
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Sets the CAN bus speed and turns on the CAN controller.
 *
 * Note:            None.
 ********************************************************************/
void CANEnable( int Channel, int BusSpeed )
{
	// CNF1 -> 0x03 = 125kB. 0x01 = 250 kB, 0x00 = 500kB

	if (BusSpeed==10) // For 125 kB
	{
		CAN2515ByteWrite(Channel, CNF1,0x03); //used to be: BusSpeed
		CAN2515ByteWrite(Channel, CNF2,0xac); //used to be: BusSpeed
		CAN2515ByteWrite(Channel, CNF3,0x07); //used to be: BusSpeed
	}
	else if (BusSpeed==7) // For 250 kB
	{
		CAN2515ByteWrite(Channel, CNF1,0x01); //used to be: BusSpeed
		CAN2515ByteWrite(Channel, CNF2,0xac); //used to be: BusSpeed
		CAN2515ByteWrite(Channel, CNF3,0x07); //used to be: BusSpeed
	}
	else if(BusSpeed==5) // For 500 kB
	{
		CAN2515ByteWrite(Channel, CNF1,0x00); //used to be: BusSpeed
		CAN2515ByteWrite(Channel, CNF2,0xac); //used to be: BusSpeed
		CAN2515ByteWrite(Channel, CNF3,0x07); //used to be: BusSpeed

	}
	else if (BusSpeed==1) // For 1000 kB
	{
		CAN2515ByteWrite(Channel, CNF1,0x00); //used to be: BusSpeed
		CAN2515ByteWrite(Channel, CNF2,0x91); //used to be: BusSpeed
		CAN2515ByteWrite(Channel, CNF3,0x03); //used to be: BusSpeed
	}

	//Interrupt on RXB0 - CANINTE
	CAN2515ByteWrite(Channel, CANINTE,0x03); // Interrupts are on

	//Set NORMAL mode
	CAN2515ByteWrite(Channel, CANCTRL,REQOP_NORMAL  | CLKOUT_ENABLED);

	CAN2515ByteRead(Channel, CANSTAT); //dummy read to give 2515 time to switch to normal mode

	if( (CAN2515ByteRead(Channel, CANSTAT) & 0xE0) != OPMODE_NORMAL )
		CAN2515ByteWrite(Channel, CANCTRL,REQOP_NORMAL | CLKOUT_ENABLED);
}
/*********************************************************************
 * Function:        void InitializeCAN( int Channel , int BusSpeed, UINT16 Mask, UINT16 Flt[6])
 *
 * PreCondition:    SPI port configured
 *
 * Input:			Channel: SPI channel number, 1 based
 *					BusSpeed: Bus Speed code: CAN_1000kbps, CAN_500kbps
 *							CAN_250kbps, or CAN_125kbps
 *					Mask: Mask used for both receive buffers.
 *					Flt: Array of filters.  
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Reset CAN module. Setup MCP2515 for filtering none-extended IDs, 
 *					and to accept all extended IDs.
 *
 * Note:            None.
 *
 * Example:         InitializeCAN(0,CAN_125kbps)
 ********************************************************************/
void InitializeCAN( int Channel , int BusSpeed, UINT16 Mask, UINT16 Flt[6])
{
	CANReset(Channel);
	CAN2515SetRXB0Filters(Channel, Mask, Flt);
	CAN2515SetRXB1Filters(Channel, Mask, &Flt[2]);
	CANEnable(Channel, BusSpeed);
}
/*********************************************************************
 * Function:        void InitializeCANExtended( int Channel , int BusSpeed, UINT32 Mask, UINT32 Flt)
 *
 * PreCondition:    SPI port configured
 *
 * Input:			Channel: SPI channel number, 1 based
 *					BusSpeed: Bus Speed code: CAN_1000kbps, CAN_500kbps
 *							CAN_250kbps, or CAN_125kbps
 *					Mask: Mask used for both receive buffers.
 *					Flt: Array of filters.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		Reset the CAN module. Setup the Rx filters to filter 
 *					extended IDs, and to deny all none-extended IDs.
 *
 * Note:            None.
 *
 * Example:         InitializeCAN(0,CAN_125kbps, 0, 0) (no filter)
 ********************************************************************/
void InitializeCANExtended( int Channel , int BusSpeed, UINT32 Mask, UINT32 Flt[6])
{
	CANReset(Channel);
	CAN2515ByteWrite(Channel, RXB0CTRL, 0x02<<5); //Receive only extended id messages to buffer 0
	CAN2515ByteWrite(Channel, RXB1CTRL, 0x02<<5); //Receive only extended id messages to buffer 1
	CAN2515SetRXB0FiltersEx(Channel, Mask, Flt);
	CAN2515SetRXB1FiltersEx(Channel, Mask, &Flt[2]);
	CANEnable(Channel, BusSpeed);
}
/*********************************************************************
 * Function:        BOOL CANTxReady( int Channel )
 *
 * PreCondition:    CAN initialized
 *
 * Input:			Channel: SPI channel number, 1 based
 *
 * Output:          Return true if the CAN controller Transmit buffer is available
 *
 * Side Effects:    None
 *
 * Overview:		Application can check is Tx buffer is available before
 *					calling CANSendMSg.
 *
 * Note:            None.
 *
 * Example:			CANTxReady(0);
 ********************************************************************/
inline Bool CANTxReady( int Channel )
{
	return (ReadStatus2515(Channel)&0x04) == 0;
}
/*********************************************************************
 * Function:        BOOL CANRxReady( int Channel )
 *
 * PreCondition:    CAN initialized
 *
 * Input:			Channel: SPI channel number, 1 based
 *
 * Output:          Return true if the CAN controller Receive buffer is Full
 *
 * Side Effects:    None
 *
 * Overview:		Application can check is Rx buffer is full before
 *					calling CANGetMSg.
 *
 * Note:            None.
 *
 * Example:			CANRxReady(0);
 ********************************************************************/
inline Bool CANRxReady( int Channel )
{
	return (ReadStatus2515(Channel)&0x3) != 0;
}
/*********************************************************************
 * Function:        void config_dpi204(void)
 *
 * PreCondition:    None
 *
 * Input:			None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:		This function setups the Evk1100 display settings for the SPI interface
 *
 * Note:            None.
 *
 * Example:         config_dpi204();
 ********************************************************************/
void config_dpi204(void){
static const gpio_map_t DIP204_SPI_GPIO_MAP =
{
	{DIP204_SPI_SCK_PIN,  DIP204_SPI_SCK_FUNCTION },  // SPI Clock.
	{DIP204_SPI_MISO_PIN, DIP204_SPI_MISO_FUNCTION},  // MISO.
	{DIP204_SPI_MOSI_PIN, DIP204_SPI_MOSI_FUNCTION},  // MOSI.
	{DIP204_SPI_NPCS_PIN, DIP204_SPI_NPCS_FUNCTION}   // Chip Select NPCS.
	};

	spi_options_t spiOptions2 =
	{
	.reg          = DIP204_SPI_NPCS,
	.baudrate     = 1000000,
	.bits         = 8,
	.spck_delay   = 0,
	.trans_delay  = 0,
	.stay_act     = 1,
	.spi_mode     = 0,
	.modfdis      = 1
	};
	gpio_enable_module(DIP204_SPI_GPIO_MAP,
	sizeof(DIP204_SPI_GPIO_MAP) / sizeof(DIP204_SPI_GPIO_MAP[0]));

	spi_initMaster(DIP204_SPI, &spiOptions2);
	spi_selectionMode(DIP204_SPI, 0, 0, 0);
	spi_enable(DIP204_SPI);
	spi_setupChipReg(DIP204_SPI, &spiOptions2, FOSC0);
}
