/*
 * CFile1.c
 *
 * Created: 2016-11-10 12:12:22
 *  Author: petterhg
 */ 
/********************************************************
Name : main.c

CAN example

**********************************************************/

#include <asf.h>
#include "can.h"

//Parameter for ADC
/* Connection of the temperature sensor */
#define EXAMPLE_ADC_TEMPERATURE_CHANNEL     0
#define EXAMPLE_ADC_TEMPERATURE_PIN         AVR32_ADC_AD_0_PIN
#define EXAMPLE_ADC_TEMPERATURE_FUNCTION    AVR32_ADC_AD_0_FUNCTION

/* Connection of the light sensor */
#define EXAMPLE_ADC_LIGHT_CHANNEL           2
#define EXAMPLE_ADC_LIGHT_PIN               AVR32_ADC_AD_2_PIN
#define EXAMPLE_ADC_LIGHT_FUNCTION          AVR32_ADC_AD_2_FUNCTION

/* Connection of the potentiometer */
#define EXAMPLE_ADC_POTENTIOMETER_CHANNEL   1
#define EXAMPLE_ADC_POTENTIOMETER_PIN       AVR32_ADC_AD_1_PIN
#define EXAMPLE_ADC_POTENTIOMETER_FUNCTION  AVR32_ADC_AD_1_FUNCTION

/** GPIO pin/adc-function map. */
const gpio_map_t ADC_GPIO_MAP = {
	#if defined(EXAMPLE_ADC_TEMPERATURE_CHANNEL)
	{EXAMPLE_ADC_TEMPERATURE_PIN, EXAMPLE_ADC_TEMPERATURE_FUNCTION},
	#endif
	#if defined(EXAMPLE_ADC_LIGHT_CHANNEL)
	{EXAMPLE_ADC_LIGHT_PIN, EXAMPLE_ADC_LIGHT_FUNCTION},
	#endif
	#if defined(EXAMPLE_ADC_POTENTIOMETER_CHANNEL)
	{EXAMPLE_ADC_POTENTIOMETER_PIN, EXAMPLE_ADC_POTENTIOMETER_FUNCTION}
	#endif
};


//Parameter for CAN bus
#define CAN_1000kbps 1
#define CAN_500kbps 5
#define CAN_250kbps 7
#define CAN_125kbps 10

UINT32 Ident;
UINT8 msg[8], mSize;
UINT32 ourID = 0x01205;

void displaySensors(int, int, int,int,int);
void displayReceiveMsg();

int main(void) {
	/*Variable*/


	
	UINT32 Mask = 0xff00;
	UINT32 flt = 0x1200;
	UINT32 Flt[] = {flt,flt,flt,flt,flt,flt};
	int Channel = 0;
	
	signed short adc_value_temp  = -1;
	signed short adc_value_light = -1;
	signed short adc_value_pot   = -1;
	
	/* Init system clocks */
	sysclk_init();
	
	/* Assign and enable GPIO pins to the ADC function. */
	gpio_enable_module(ADC_GPIO_MAP, sizeof(ADC_GPIO_MAP) /
	sizeof(ADC_GPIO_MAP[0]));
	
		/* Configure the ADC peripheral module.
	 * Lower the ADC clock to match the ADC characteristics (because we
	 * configured the CPU clock to 12MHz, and the ADC clock characteristics are
	 *  usually lower; cf. the ADC Characteristic section in the datasheet). */
	AVR32_ADC.mr |= 0x1 << AVR32_ADC_MR_PRESCAL_OFFSET;
	//Set the resolution to 10bits, 0-->10, 1-->8
	AVR32_ADC.mr |= 0x0 << AVR32_ADC_MR_LOWRES_OFFSET;
	adc_configure(&AVR32_ADC);
	
	adc_enable(&AVR32_ADC, EXAMPLE_ADC_TEMPERATURE_CHANNEL);
	adc_enable(&AVR32_ADC, EXAMPLE_ADC_LIGHT_CHANNEL);
	adc_enable(&AVR32_ADC, EXAMPLE_ADC_POTENTIOMETER_CHANNEL);
	
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
	
	/*Initializes communication can*/
	InitializeCANExtended(Channel, CAN_250kbps, Mask, Flt);
	
	
	/*First message*/
	dip204_set_cursor_position(1,1);
	dip204_printf_string("Heja");
	dip204_hide_cursor();
	

	while(1){

		//Clear memory contents
		ClearMessages(msg);
		
		/*Read sensors values*/
		adc_start(&AVR32_ADC);
		adc_value_temp = adc_get_value(&AVR32_ADC,EXAMPLE_ADC_TEMPERATURE_CHANNEL);
		adc_value_light = adc_get_value(&AVR32_ADC,EXAMPLE_ADC_LIGHT_CHANNEL);
		adc_value_pot = adc_get_value(&AVR32_ADC,EXAMPLE_ADC_POTENTIOMETER_CHANNEL);	

		
		int ValueSummer[20][4] = {0};
		UINT32 IDSummer[20] = {0};
		int counter = 0;
		int temp_avg = 0;
		int light_avg = 0;
		int pot_avg = 0;
		int i = 0;
		int timeCounter=0;
		bool flag = FALSE;
		//Read any message available
		if(CANRxReady(0)){
			while(timeCounter < 120)
			{
				if( CANGetMsg(0, &Ident, msg, &mSize )) // Gets message and returns //TRUE if message received.
				{			
				
					if (counter==0){
						IDSummer[counter] = Ident; //Identity
						ValueSummer[counter][1] = (msg[0] << 8) | msg[1]; //Temperature
						ValueSummer[counter][2] = (msg[2] << 8) | msg[3]; //Light
						ValueSummer[counter][3] = msg[4];
						counter++;	
					}
					else{
						flag=FALSE;
						for (i=0; i<counter; i++){
							if (IDSummer[i] == Ident){
								flag=TRUE;
								break;
							}
						}
						
						if (flag==FALSE)
						{
								IDSummer[counter] = Ident; //Identity
								ValueSummer[counter][1] = (msg[0] << 8) | msg[1]; //Temperature
								ValueSummer[counter][2] = (msg[2] << 8) | msg[3]; //Light
								ValueSummer[counter][3] = msg[4];
								counter++;	
						}
					}
				}
				
			

			delay_ms(10);
			timeCounter++;
			}
			
			//Compute the mean of the temp and light (include our own value)
			for (i=0; i<counter; i++){
				temp_avg = temp_avg + ValueSummer[i][1]; 
				light_avg = light_avg + ValueSummer[i][2]; 
				pot_avg = pot_avg + ValueSummer[i][3]; 
			}
			temp_avg=(temp_avg+adc_value_temp)/(counter+1);
			light_avg=(light_avg+adc_value_light)/(counter+1);
			pot_avg=(pot_avg+adc_value_pot)/(counter+1);
			
			//Print everything	
			displaySensors(counter,adc_value_temp, temp_avg,adc_value_light,light_avg);//Display and convert the sensors values
			
			// Send messages if possible (every 500ms)
			if(CANTxReady(0) && timeCounter==50)
			{
				// Channel, Identifier (max 0x1fffffff (29 bits)), Message, Number of bytes, R //or 0 (Remote frame or no remote frame).
				CANSendMsg( 0, ourID, msg, 5, 0 );
				delay_ms(10);
			}	
						
		}//if(CANRxReady(0))
		
		

		}//While(1)
	return 0;

}


void displaySensors(int nbID, int temperature, int avgTemp,int light,int avgLight){
		
		
			/*Print on the display*/
			dip204_clear_display();//Clear all
			
			//ID
			dip204_set_cursor_position(1,1);
			dip204_printf_string("#Node:%d",nbID);
			
			//temperature
			dip204_set_cursor_position(1,2);
			dip204_printf_string("Temp: %d",temperature);
			dip204_set_cursor_position(12,2);
			dip204_printf_string("%d", avgTemp);
			
			//Light
			dip204_set_cursor_position(1,3);
			dip204_printf_string("Light: %d",light);
			dip204_set_cursor_position(12,3);
			dip204_printf_string("%d", avgLight);
			
			return;
			
}

void displaySensorsskldjf(signed short adc_value_temp,signed short  adc_value_light,signed short  adc_value_pot){
		
			UINT16 temperature = 0;
			UINT16 light = 0;
			UINT16 pot = 0;
			int absPot = 0;
			
			light = adc_value_temp;//Assign the value to 16 bites
			//Separate the 10 bits into 2 and 8;
			msg[0] = light >> 8;
			msg[1] = light & 0x00ff;
			
			light = adc_value_light;//Assign the value to 16 bites
			//Separate the 10 bits into 2 and 8;
			msg[2] = light >> 8;
			msg[3] = light & 0x00ff;
			
			pot = (adc_value_pot*255)/1023;//Change the range form 0-1023 to 0-255 and convert to bytes
			msg[4]=pot;
			
			/*Print on the display*/
			dip204_clear_display();//Clear all
			
			//temperature
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
			return;
			
			
}

void displayReceiveMsg(){
			
					UINT16 temperature = 0;
					UINT16 light = 0;
					UINT16 pot = 0;
					int absPot = 0;
	dip204_clear_display();
	/*Convert to one value*/
				/*temperature*/
				/*	
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
				*/			
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
				/*			
				//ID and DLC
				dip204_set_cursor_position(1,4);
				dip204_printf_string("Id: ");
				dip204_set_cursor_position(4,4);
				dip204_printf_string("%x", Ident);
				dip204_set_cursor_position(13,4);
				dip204_printf_string("DLC:");
				dip204_set_cursor_position(17,4);
				dip204_printf_string("%x", mSize);
				*/
}