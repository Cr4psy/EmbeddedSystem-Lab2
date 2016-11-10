/********************************************************
Name : main.c

CAN example

**********************************************************/

#include <asf.h>
#include "can.h"

#define CAN_1000kbps 1
#define CAN_500kbps 5
#define CAN_250kbps 7
#define CAN_125kbps 10

UINT32 Ident;
UINT8 msg[8], mSize;

int mainLab2(void) {
	//spidatareadpointer=&spidataread;
	pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP);
	
	// Configures the MCP2515 SPI communication.
	config_SPI_SPARE();

	// Enables receive interrupts.
	Disable_global_interrupt();
	INTC_init_interrupts();
	Enable_global_interrupt();
	
	// Delay to let the Oscillator get started
	delay_init( FOSC0 );
	
	// Initializes the display
	config_dpi204();
	dip204_init(100,1);
	dip204_clear_display();
	
	/*Initializes variable for communication can*/
	UINT16 Mask = 0; 
	UINT16 flt = 0;
	UINT16 Flt[] = {flt,flt,flt,flt,flt,flt};
	InitializeCAN(0, CAN_250kbps, Mask, Flt);
	
	

	
	/*First message*/
	dip204_set_cursor_position(1,1);
	dip204_printf_string("Heja");
	dip204_hide_cursor();
	
	/*Variable*/
	UINT16 temperature = 0;
	UINT16 light = 0;
	UINT16 pot = 0;
	int absPot = 0;

	while(1){
		temperature = 0;
		light = 0;
		pot = 0;
		//Clear memory contents
		ClearMessages(msg);
		
		//Read any message available
		if(CANRxReady(0)){
			if( CANGetMsg(0, &Ident, msg, &mSize )) // Gets message and returns //TRUE if message received.
			{	
				if (Ident == 0x01000)//Select only message with ID 0x0100
				{

				// Evk1100PrintDisplay prints 4 message values, the Identifier and the data size on the display
				dip204_clear_display();
					
				/*Convert to one value*/
				/*temperature*/
				temperature = (msg[0] << 8) | msg[1];
				dip204_set_cursor_position(1,1);
				dip204_printf_string("Temp:");
				dip204_set_cursor_position(8,1);
				dip204_printf_string("%d", (int)temperature);
				
				//Light
				light = (msg[2] << 8) | msg[3];
				dip204_set_cursor_position(1,2);
				dip204_printf_string("Light:");
				dip204_set_cursor_position(8,2);
				dip204_printf_string("%d", (int)light);
				
				//Potentiometer
				pot = msg[4];
				absPot = ((int)pot*100)/255;
				dip204_set_cursor_position(1,3);
				dip204_printf_string("Pot:");
				dip204_set_cursor_position(5,3);
				dip204_printf_string("%d", pot);
				dip204_set_cursor_position(9,3);
				dip204_printf_string("Abs:");
				dip204_set_cursor_position(13,3);
				dip204_printf_string("%d %%", absPot);
				
				//ID and DLC
				dip204_set_cursor_position(1,4);
				dip204_printf_string("Id: ");
				dip204_set_cursor_position(4,4);
				dip204_printf_string("%x", Ident);
				dip204_set_cursor_position(13,4);
				dip204_printf_string("DLC:");
				dip204_set_cursor_position(17,4);
				dip204_printf_string("%x", mSize);
						
				
				delay_ms(500);
				}
			}
		}
	}
	return 0;
}