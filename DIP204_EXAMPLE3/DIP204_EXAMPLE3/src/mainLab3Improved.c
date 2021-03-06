/*
 * CFile1.c
 *
 * Created: 2016-11-14 12:11:26
 *  Author: clerc
 */ 
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
UINT32 ourID = 0x01206;

void displaySensors(int, int, int,int,int);
void displayReceiveMsg();

int main(void) {
	/*Variable*/
	int i = 0;
	bool flag = false;
	int temp_avg = 0;
	int light_avg = 0;
	int pot_avg = 0;
	int timerCounter = 0;//Count the time
	int timerTime = 10;//time between the execution of loop while
	int counter = 0; //Count the number of ID
	int ValueSummer[20][3] = {0};
	UINT32 IDSummer[20] = {0};

	//CAN Communication variable
	UINT32 Mask = 0xff00;//Mask
	UINT32 flt = 0x1200;//Filter
	UINT32 Flt[] = {flt,flt,flt,flt,flt,flt};
	int Channel = 0;
	
	//Value of the sensors
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
	//Set the frequency of the analog inputs
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
	
	/*Initializes communication can*/
	InitializeCANExtended(Channel, CAN_250kbps, Mask, Flt);
	
	// Initializes the display
	config_dpi204();
	dip204_init(100,1);
	dip204_clear_display();
		/*First message*/
		dip204_set_cursor_position(1,1);
		dip204_printf_string("Heja");
		dip204_set_cursor_position(1,2);
		dip204_printf_string("Petter and Anthony");
		dip204_hide_cursor();
		delay_ms(2000);	

	while(1){
		
		
		/*READ ADC VALUE*/
		if (timerCounter%(300/timerTime) == 0)//Read adc every 300ms
		{
			//Clear memory contents
			ClearMessages(msg);
			/*Read sensors values*/
			adc_start(&AVR32_ADC);
			adc_value_temp = adc_get_value(&AVR32_ADC,EXAMPLE_ADC_TEMPERATURE_CHANNEL);
			adc_value_light = adc_get_value(&AVR32_ADC,EXAMPLE_ADC_LIGHT_CHANNEL);
			adc_value_pot = (adc_get_value(&AVR32_ADC,EXAMPLE_ADC_POTENTIOMETER_CHANNEL)*255)/1023;//Change the range form 0-1023 to 0-255 and convert to bytes
		}
		
		/*PRINT AND RESET*/	
		//Erase the array and count again
		//Print on the display
		if (timerCounter%(1500/timerTime) == 0)//Done every 1500ms
		{
			
			//Compute the mean of the temp and light (include our own value)
			for (i=0; i<counter; i++){
				temp_avg = temp_avg + ValueSummer[i][0];
				light_avg = light_avg + ValueSummer[i][1];
				pot_avg = pot_avg + ValueSummer[i][2];
			}
			//Compute the average and add the value of the actual board.
			temp_avg=(temp_avg+adc_value_temp)/(counter+1);
			light_avg=(light_avg+adc_value_light)/(counter+1);
			pot_avg=(pot_avg+adc_value_pot)/(counter+1);
			
			//Print everything
			displaySensors(counter,adc_value_pot,pot_avg,adc_value_light,light_avg);//Display and convert the sensors valuesadc_value_temp, temp_avg
			
			/*RESET*/		
			//Reset the values and ID
			int ValueSummer[256][3] = {0};
			UINT32 IDSummer[256] = {0};
			counter=0;//Reset the number of ID
			temp_avg=0;
			light_avg=0;
			pot_avg=0;
					
			
		}
		
		/*READ BUS*/
		if(CANRxReady(0)){//If able to connect to the bus
			if(CANGetMsg(0, &Ident, msg, &mSize )) // Gets message and returns //TRUE if message received.
			{				
				if (counter==0){//If no ID in the array
					IDSummer[counter] = Ident; //Identifier
					ValueSummer[counter][0] = (msg[0] << 8) | msg[1]; //Temperature
					ValueSummer[counter][1] = (msg[2] << 8) | msg[3]; //Light
					ValueSummer[counter][2] = msg[4];//Potentiometer
					counter++;//Nb of ID
				}
				else{//If already some ID in the array
					flag=FALSE;
					for (i=0; i<counter; i++){//Go through all the id values
						if (IDSummer[i] == Ident){//If similar ID, refresh value and go out of for loop
							ValueSummer[i][0] = (msg[0] << 8) | msg[1]; //Temperature
							ValueSummer[i][1] = (msg[2] << 8) | msg[3]; //Light
							ValueSummer[i][2] = msg[4];//pot
							flag=TRUE;
							break;
						}
					}
					
					if (flag==FALSE)//If it�s the first time that the id is detected, so we add the value to the array
					{
						IDSummer[counter] = Ident; //Identifier
						ValueSummer[counter][0] = (msg[0] << 8) | msg[1]; //Temperature
						ValueSummer[counter][1] = (msg[2] << 8) | msg[3]; //Light
						ValueSummer[counter][2] = msg[4];//pot
						counter++;//Nb of ID
					}
				}//else
			}//CANGetMsg()		
		}//CANRxReady()
		
		
		/*SEND BUS*/
		// Send messages if possible (every 500ms)
		if (timerCounter%(500/timerTime)==0)
		{
		
			if(CANTxReady(0))
			{
				/*Convert value to bytes*/
				//Light
				//Separate the 10 bits into 2 and 8;
				msg[0] = adc_value_temp >> 8;
				msg[1] = adc_value_temp & 0x00ff;
			
				//Temperature
				//Separate the 10 bits into 2 and 8;
				msg[2] = adc_value_light >> 8;
				msg[3] = adc_value_light & 0x00ff;
			
				msg[4]=adc_value_pot;//Change the range form 0-1023 to 0-255 and convert to bytes
			
				// Channel, Identifier (max 0x1fffffff (29 bits)), Message, Number of bytes, R //or 0 (Remote frame or no remote frame).
				CANSendMsg( 0, ourID, msg, 5, 0 );
			}
		}
		
		
		delay_ms(timerTime);//Force the while loop to run max every timerTime ms
		timerCounter++;
	}
	
	//###################################################################################################
	//END WHILE(1)

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
			dip204_printf_string("Temp:");
			dip204_set_cursor_position(8,2);
			dip204_printf_string("%d",temperature);
			dip204_set_cursor_position(12,2);
			dip204_printf_string("%d", avgTemp);
			
			//Light
			dip204_set_cursor_position(1,3);
			dip204_printf_string("Light:");
			dip204_set_cursor_position(8,3);
			dip204_printf_string("%d",light);
			dip204_set_cursor_position(12,3);
			dip204_printf_string("%d", avgLight);
			
			//MyID
			dip204_set_cursor_position(1,4);
			dip204_printf_string("MyID: %d",ourID);
			dip204_set_cursor_position(12,4);
			dip204_printf_string("0x%x",ourID);

			
			return;
			
}



