/*
 * CFile1.c
 *
 * Created: 2016-11-18 12:06:25
 *  Author: clerc
 */ 
// This program turns on and off LED1 on the EVK1100 board.
// The program uses interrupt on falling edge on pushbutton 0 (PB0).
#include <asf.h>
#define Switch1 88 //Switch PB0 connected to PX16, i.e. GPIO nr 88
volatile int x=1;
__attribute__((__interrupt__)) static void interrupt( void )
{
	if (x==1)
	{
		LED_On(LED0);
		x=2;
	}
	else if (x==2)
	{
		LED_Off(LED0);
		x=1;
	}
	// Clears the interrupt flag
	gpio_clear_pin_interrupt_flag(Switch1);
}
int main(void) {
	board_init();
	// Here are the interrupts enabled
	INTC_init_interrupts();
	/*interrupt stands for the interrupt function after _attribute_,
	AVR32_GPIO_IRQ_0+88/8 stands for the interrupt line (88 = pin number) and
	AVR32_INTC_INT0 for the interrupt level*/
	INTC_register_interrupt(&interrupt,(AVR32_GPIO_IRQ_0+88/8),AVR32_INTC_INT0);
	// Enables gpio control for the pin
	gpio_enable_gpio_pin(Switch1);
	// Sets a specific respons time for the interrupt
	gpio_enable_pin_glitch_filter(Switch1);
	// Enables a certain pin and set how it should react
	gpio_enable_pin_interrupt(Switch1,GPIO_FALLING_EDGE);
	// Enables global interrupts
	Enable_global_interrupt();
	while (1){}
	return 0;
}