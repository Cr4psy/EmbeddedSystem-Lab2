/********************************************************
Name : main.c

CAN example

Petter Hultin and Anthony Clerc

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
UINT32 ourID = 0x01201;

void displaySensors(int, int, int,int,int, int);
void displayReceiveMsg();
void LEDState(UINT8, int); //Change the led state
void Button(int);
void readTheBus();

int timerTime = 10;//time between the execution of loop while
UINT8 flagStatus = 0;/*	00 : Normal
						01 : Warning status 
						10 : Emergency status
						*/

int ValueSummer[20][6] = {0};//Value received from node (temp, light, pot and status)
UINT32 IDSummer[20] = {0};//Each ID received
int counter = 0; //Count the number of ID


/*INTERRUPT*/
#define Switch1 29 //TWI-SDA connected to PA29, i.e. GPIO nr 29
//Interrupt each time a message is received
__attribute__((__interrupt__)) static void interrupt( void )
{
	/*READ BUS*/
	readTheBus();
	// Clears the interrupt flag
	gpio_clear_pin_interrupt_flag(Switch1);
}

int main(void) {
	/*Variable*/

	int temp_avg = 0;
	int light_avg = 0;
	int pot_avg = 0;
	int nodeNight = 0;
	int nodeFault = 0;
	int nodeWarm = 0;
	int timerCounter = 0;//Count the time



		
	//Problems flag
	UINT8 flagState = 0;/*	001 : night
							010 : warm temperature
							100 : fault
							*/


	//CAN Communication variable
	UINT32 Mask = 0x1fff00;//Mask
	UINT32 flt =  0x001200;//Filter
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
	
	gpio_enable_gpio_pin(88);
	
	//spidatareadpointer=&spidataread;
	pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP);
	
	// Configures the MCP2515 SPI communication.
	config_SPI_SPARE();

	// Enables receive interrupts.
	Disable_global_interrupt();
	INTC_init_interrupts();
		/*interrupt stands for the interrupt function after _attribute_,
	AVR32_GPIO_IRQ_0+88/8 stands for the interrupt line (88 = pin number) and
	AVR32_INTC_INT0 for the interrupt level*/
	INTC_register_interrupt(&interrupt,(AVR32_GPIO_IRQ_0+Switch1/8),AVR32_INTC_INT0);
	// Enables gpio control for the pin
	gpio_enable_gpio_pin(Switch1);
	// Sets a specific respons time for the interrupt
	gpio_enable_pin_glitch_filter(Switch1);
	// Enables a certain pin and set how it should react
	gpio_enable_pin_interrupt(Switch1,GPIO_PIN_CHANGE);
	// Enables global interrupts
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
		dip204_printf_string("Hej");
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
			adc_value_pot = ((adc_get_value(&AVR32_ADC,EXAMPLE_ADC_POTENTIOMETER_CHANNEL)*255)/1023);//Change the range form 0-1023 to 0-255 and convert to bytes
		}
		
		/*PRINT AND RESET*/	
		//Erase the array and count again
		//Print on the display
		if (timerCounter%(1500/timerTime) == 0)//Done every 1500ms
		{
			/*RESET*/
			int i = 0;
			temp_avg = 0;
			light_avg = 0;
			pot_avg = 0;
			nodeNight = 0;
			nodeFault = 0;
			nodeWarm = 0;
			
			//Compute the mean of the temp and light (include our own value)
			for (i=0; i<counter; i++){
				if (ValueSummer[i][5]!=TRUE)//If the node is not in error
				{
					temp_avg = temp_avg + ValueSummer[i][0];
					light_avg = light_avg + ValueSummer[i][1];
					pot_avg = pot_avg + ValueSummer[i][2];
					nodeNight=nodeNight+ValueSummer[i][3];//Sum of night node
					nodeWarm=nodeWarm+	ValueSummer[i][4];//Sum of warm node
				}
				nodeFault=nodeFault+ValueSummer[i][5];//# of faulty nodes			
			}
			//Compute the average and add the value of the actual board.
			//-nodeFault
			temp_avg=(temp_avg+adc_value_temp)/(counter+1-nodeFault);
			light_avg=(light_avg+adc_value_light)/(counter+1-nodeFault);
			pot_avg=(pot_avg+adc_value_pot)/(counter+1-nodeFault);
			
			/*Problems states for this node*/
			flagState=0;//Reset flags
			//Night/Day
			if (adc_value_light<(0.5*light_avg))
			{
				flagState = (flagState | 0b001);
			}
			//Temperature Warm
			if (adc_value_temp>(1.15*temp_avg))
			{
				flagState = (flagState | 0b010);
			}
			//Cold temperature -> Fault
			if (adc_value_temp<(0.5*temp_avg))
			{
				flagState = (flagState | 0b100);
			}
			
			
			/*Status*/
			nodeNight=nodeNight+(flagState&0b001 == 1);//Add this node result
			nodeWarm=nodeWarm+((flagState&0b010)>>1 == 1);//Add this node result
			nodeFault=nodeFault+((flagState&0b100)>>2 == 1);//Add this node result
						
			if (nodeWarm==1)//Warning state
			{
				flagStatus= flagStatus | 0b001;
			}
			else if (nodeWarm > 1)//Emergency state
			{
				flagStatus = flagStatus | 0b010;
			}
			else//Normal status
			{
				flagStatus =  flagStatus&0b010;//Set to zero only warning and  no emergency (Need to press the button)
			}
			
			
			if (nodeFault>0)//Faulty state
			{
				flagStatus = flagState | 0b100;
			}
			else
			{
				flagStatus = flagStatus & 0b011;
			}

			
			
			//Print everything
			displaySensors(counter+1,adc_value_temp,temp_avg,adc_value_light,light_avg, nodeNight);//Display and convert the sensors valuesadc_value_temp, temp_avg, +1 to take our node	
			
			/*RESET*/
			//Reset the values and ID
			counter = 0;//Reset the number of ID				
		}
		
		LEDState(flagState, timerCounter);//Change the state of the led (every 10 ms)
		Button(nodeWarm);//Check if the button is pressed to acknowledge the alarm
		
		/*READ BUS*/
		//The bus is read in the interrupt function, only when we receive a message
		
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
				
				//Potentiometer
				msg[4]=adc_value_pot;//Change the range form 0-1023 to 0-255 and convert to bytes
						
				//Problems flag
				msg[5]=flagState;//night / warm / fault
				
				// Channel, Identifier (max 0x1fffffff (29 bits)), Message, Number of bytes, R //or 0 (Remote frame or no remote frame).
				CANSendMsg( 0, ourID, msg, 6, 0 );
			}
		}
		
	
		delay_ms(timerTime);//Force the while loop to run max every timerTime ms
		timerCounter++;
	}
	
	//###################################################################################################
	//END WHILE(1)

	return 0;
}

/*READ BUS*/
void readTheBus(){
		int i = 0;
		bool flag = false;
	
	/*READ BUS*/
		if(CANRxReady(0)){//If able to connect to the bus
			if(CANGetMsg(0, &Ident, msg, &mSize )) // Gets message and returns //TRUE if message received.
			{				
				if (counter==0){//If no ID in the array
					IDSummer[counter] = Ident; //Identifier
					ValueSummer[counter][0] = (msg[0] << 8) | msg[1]; //Temperature
					ValueSummer[counter][1] = (msg[2] << 8) | msg[3]; //Light
					ValueSummer[counter][2] = msg[4];//Potentiometer
					ValueSummer[counter][3] = (msg[5] & 0b001);//Night
					ValueSummer[counter][4] = (msg[5] & 0b010)>>1;//Too warm
					ValueSummer[counter][5] = (msg[5] & 0b100)>>2;//Faulty
					counter++;//Nb of ID
				}
				else{//If already some ID in the array
					flag=FALSE;
					for (i=0; i<counter; i++){//Go through all the id values
						if (IDSummer[i] == Ident){//If similar ID, refresh value and go out of for loop
							ValueSummer[i][0] = (msg[0] << 8) | msg[1]; //Temperature
							ValueSummer[i][1] = (msg[2] << 8) | msg[3]; //Light
							ValueSummer[i][2] = msg[4];//pot
							ValueSummer[i][3] = (msg[5] & 0b01);//Night
							ValueSummer[i][4] = (msg[5] & 0b010)>>1;//Too warm
							ValueSummer[i][5] = (msg[5] & 0b100)>>2;//Faulty
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
						ValueSummer[counter][3] = (msg[5] & 0b01);//Night
						ValueSummer[counter][4] = (msg[5] & 0b010)>>1;//Too warm
						ValueSummer[counter][5] = (msg[5] & 0b100)>>2;//Faulty
						counter++;//Nb of ID
					}
				}//else
			}//CANGetMsg()		
		}//CANRxReady()
}

//BUTTON
//Reset emergency mode
void Button(int nodeWarm)
{
	if ((gpio_get_pin_value(88)==0) && (nodeWarm<2))
	{	

		flagStatus = flagStatus & 0b101;//RESET emergency
	}
	
}

//Manage the led STATE
void LEDState(UINT8 flagState, int timerCounter){//Night / warm / fault, warning/ emergency / faulty
	//State
	LED_Off(0b111);
	LED_On(flagState);
	
	/*STATUS*/
	if (flagStatus == 0b000)//Normal
	{
		LED_On(0b1<<5);//Turn on led 5
		LED_Off(0b101<<4);//Turn off led 4 and 6
	}
	else{
		LED_Off(0b1<<5);
		if (((flagStatus >>1)& 0b01) == 1)//Emergency
		{
			if(timerCounter%(500/timerTime)==0){
				LED_Toggle(0b101<<4);//Led 4 6 Blink every x second
			}
		}	
		else if (((flagStatus>>0)& 0b01) == 1)//Warning
		{
			LED_Off(0b1<<6);
			if(timerCounter%(500/timerTime)==0){
				LED_Toggle(0b1<<4);//Blink every second
			}
		}
		
	} 
	
	/*ONE NODE IS FAULTY*/
	if (((flagStatus >>2)& 0b01) == 1)//Faulty
	{
		if(timerCounter%(200/timerTime)==0){
			LED_Toggle(0b1<<3);//Blink every x second
		}
	}
	else{
		LED_Off(0b1<<3);
	}

}

//Write on the display
void displaySensors(int nbID, int temperature, int avgTemp,int light,int avgLight, int nodeNight){
		
		
			/*Print on the display*/
			dip204_clear_display();//Clear all
			
			//ID
			dip204_set_cursor_position(1,1);
			dip204_printf_string("#Node:%d",nbID);
			
			//Night/Day
			dip204_set_cursor_position(9,1);
			dip204_printf_string("#Daylight:%d",(nbID-nodeNight));
			
			//temperature
			dip204_set_cursor_position(1,2);
			dip204_printf_string("Temp:");
			dip204_set_cursor_position(8,2);
			dip204_printf_string("%d",temperature);
			dip204_set_cursor_position(13,2);
			dip204_printf_string("%d", avgTemp);
						
			//Light
			dip204_set_cursor_position(1,3);
			dip204_printf_string("Light:");
			dip204_set_cursor_position(8,3);
			dip204_printf_string("%d",light);
			dip204_set_cursor_position(13,3);
			dip204_printf_string("%d", avgLight);
			
			//STATUS
			
			//FAULTY	


			if (((flagStatus >>1)& 0b01) == 1)//Emergency state
			{
				dip204_set_cursor_position(1,4);
				dip204_printf_string("EMERGENCY           ");
			}
			else if (((flagStatus >>0)& 0b01) == 1)//Warning state
			{
				dip204_set_cursor_position(1,4);
				dip204_printf_string("WARNING           ");
			}
			
			if (((flagStatus >>2)& 0b01) == 1)//Faulty state
			{
				dip204_set_cursor_position(11,4);
				dip204_printf_string("FAULTY");
			}
			
			
			if (flagStatus == 0)//Normal status
			{	
				dip204_set_cursor_position(1,4);
				dip204_printf_string("                    ");
			}
			

						
			return;
			
}



