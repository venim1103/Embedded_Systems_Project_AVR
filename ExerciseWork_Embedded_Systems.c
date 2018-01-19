/*
 * ExerciseWork.c
 *
 * Created: 2.12.2015 14:13:21
 *  Authors: Ari Potinkara, Mikko Salonen
 */ 

/******************************************************************/
/*******************************************************************
 Defined libraries                                                                                               
  *****************************************************************/
/******************************************************************/

#define F_CPU 32000000

#include <avr/io.h>    		// AVR's own I/O library
#include <avr/interrupt.h>	// AVR's own library for interrupts
#include <util/delay.h>

#include <stdio.h>			// Standard C language libraries
#include <stdlib.h>
#include <string.h>
#include "my_lcd_h.h"		// The library file for the LCD

 
/******************************************************************/
/*******************************************************************
 Macros                                                                                               
 ******************************************************************/
/******************************************************************/ 

/* Add all the #Define stuff here */
/* For example, baud rates and everything related to it */

#define VALUES_MIN 0		// Minimum value for global value variables
#define VALUES_MAX 100		// Maximum value for global value variables

#define PULLUP 0b00011001 	// Pullup (totempole), rising edge

#define USART USARTC0		// Use port C for USART communication.
#define USART_PORT PORTC

#define BSCALE_VALUE 0		// To count 9600 baudrate we chose base scaling of 0 so we had to use base-select value of 207.
#define BSEL_VALUE   207

#define T 0.01				// PID timestep / timerclock1 frequency in seconds.

#define PWM_PERIOD 1280		// PWM period for getting 25 kHz frequency.
#define PWM_MIN 0
#define PWM_MAX 1280

/* Create permanent state labels, which will be printed on the LCD. 
Include some empty spaces to avoid having strange numbers and letters printed behind the labels */
#define LABEL_T_ACT "T ACT    "	 // Actual measured temperature
#define LABEL_T_REF "T REF    "	 // The reference temperature
#define LABEL_P "PID P    "		 // PID P component
#define LABEL_I "PID I    "		 // PID integral component
#define LABEL_D "PID D    "		 // PID derivative component


/*******************************************************************/
/*******************************************************************
 Structs example to store PID controller gain values
 *******************************************************************/
/*******************************************************************/

/*void init_structures(void)
{

};
*/

struct State4Machine {
	int T_m;		// State 0, the initial state, actual measured temperature
	int Tref;		// State 1, reference temperature, possible to increase/decrease value
	int P;			// State 2, P value, possible to increase/decrease value
	int I;			// State 3, I value, possible to increase/decrease value
	int D;			// State 4, D value, possible to increase/decrease value
	}state;
		
/* The values for each struct member are assigned in the beginning of the functions, where they are needed. If the values are given 
as global values, the code cannot be compiled. */

/* Notice that these numbers correspond to the "values" -vector's values (example below). Apart from state 0 (measured temp), 
it is possible to change the states' values. 
	   
	values[state.T_m] == values[0]		
	values[state.Tref] == values[1]
	values[state.P] == values[2]
	values[state.I] == values[3]
	values[state.D] == values[4]
	   
This allows the programmer to use the state struct's members to point to the "values" vector's values (memory slots) in a very easy and clear way */	
	

/******************************************************************/
/******************************************************************
Global variables                                                                                             
 ******************************************************************/
/******************************************************************/ 

volatile int16_t pid_value = 0;		// Global 

/* Global table for the different values (Measured temp, Ref temp, P,I,D) */
volatile int16_t values[5] = {0,32,10,1,0};			// Initialize all values to convenient values in the beginning (min value == 0).

/* Set the very first value of the state machine to the measured temperature. This will be the first thing that the LCD
shows when the code is uploaded to the development board and the board starts to function */
volatile int machine_state = 0;


/* For printing the duty cycle and the measured temp via UART. */
char dutycycle[] = "The duty cycle:\t";				// The text that is printed before the duty cycle value
char uart_temp[] = "The measured temperature:\t";	// The text that is printed in front of the measured temp
char tx_buffer1[20];								// Buffer, where the duty cycle's value is stored
char tx_buffer2[20];								// Buffer, where the measured temperature's value is stored


/******************************************************************/
/******************************************************************
 Function prototypes http://en.wikipedia.org/wiki/Function_prototype                                                                                 
 ******************************************************************/
/******************************************************************/ 
  
void PidController(void);	// The function for controlling the PWM duty cycle

/* The functions, which are needed for printing things via UART */
void sendString(char *text); 
void sendChar(char c);


/******************************************************************/
/******************************************************************
 Board clock initialization                                                                                           
 ******************************************************************/
/******************************************************************/
 
 /* HR pwm_led.c */
void init_CLOCK(void)
{
	OSC.CTRL |= OSC_RC32MEN_bm | OSC_RC32KEN_bm;		// Enable the internal 32MHz & 32KHz oscillators
    while(!(OSC.STATUS & OSC_RC32KRDY_bm));				// Wait for 32Khz oscillator to stabilize
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));				// Wait for 32MHz oscillator to stabilize
	DFLLRC32M.CTRL = DFLL_ENABLE_bm ;					// Enable DFLL - defaults to calibrate against internal 32Khz clock
	CCP = CCP_IOREG_gc;									// Disable register security for clock update
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;					// Switch to 32MHz clock
	OSC.CTRL &= ~OSC_RC2MEN_bm;							// Disable 2Mhz oscillator
	CLK.RTCCTRL = CLK_RTCSRC_ULP_gc | CLK_RTCEN_bm;		// 0x01
}


/******************************************************************/
/******************************************************************
 USART initialization                                                                                           
 ******************************************************************/
/******************************************************************/
 
void init_USART(void)
{
	USART_PORT.DIRSET = PIN3_bm;  					// Enable Pin 3 (TXC0) as output using port C.
	USART_PORT.DIRCLR = PIN2_bm;					// Enable Pin 2 (RXC0) as input using port C.
	
	PORTC_PIN3CTRL |= PORT_INVEN_bm;				// Enable inverted output and input for PIN 3.
	PORTC_PIN2CTRL |= PORT_INVEN_bm;				// Enable inverted output and input for PIN 2.

	USART.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc; // Set USART channel size to 8 bits and disable parity mode.

	USART.BAUDCTRLA = BSEL_VALUE;					// Save the base-select value of 207 (for 9600 baudrate) into baudrate register A.
	USART.CTRLB = USART_RXEN_bm | USART_TXEN_bm;	// Enable RX and TX transmissions.
	USART.CTRLA |= USART_RXCINTLVL_HI_gc;			// Set High-level interrupts for RX.
}


/*******************************************************************/
/******************************************************************
PWM initialization                                                                                   
  ******************************************************************/ 
/*******************************************************************/

void init_PWM(void)
{
	PORTC.DIR |= 0x01;									// Set Pin 0 as output 
	PORTC.PIN0CTRL = 0b01010011;						// Pulldown, level sense. Bit7 = 1 --> Full power when (Pid_value == 0)
	
	TCC0.PER = PWM_PERIOD;
	TCC0.CCABUF = 0;									// Start value is zero
													
	TCC0.CTRLB |= TC0_CCAEN_bm | TC_WGMODE_DS_TB_gc;	// Enable CCAEN. Waveform generation mode is set to Dual-slope PWM (Top and Bottom)
													
	TCC0.CTRLA |= TC_CLKSEL_DIV1_gc;					// Prescaler: Clk
}


/******************************************************************/
 /******************************************************************
 ADC initialization                                                                                               
 ******************************************************************/
 /*****************************************************************/ 

void init_ADC(void) 
{
	/* Set port A as input */
	PORTA.DIR = 0x00;	
	//PORTA.PIN7CTRL |= INPUT_DISABLE_gc;
	/* Configure pin 7 to disable digital input buffer */
	PORTA.PIN6CTRL = 0b00000111;
	
	/* Set pin 7 to totem pole, that senses on both edges */
////	PORTA.PIN6CTRL = 0x00;
	/* Set up ADC A to have signed conversion mode and 12 bit resolution. */ 
	ADCA.CTRLB |= ADC_RESOLUTION_12BIT_gc ;
	ADCA.CTRLB |= ADC_CONMODE_bm; 
	/* The ADC has different voltage reference options, controlled by the REFSEL bits in the*/
	/* REFCTRL register. Here the internal reference is selected*/
////	ADCA.REFCTRL |= ADC_REFSEL_INTVCC_gc | ADC_CONMODE_bm;
	ADCA.REFCTRL = 0b00010011;
	/* The clock into the ADC decide the maximum sample rate and the conversion time, and */
	/* this is controlled by the PRESCALER bits in the PRESCALER register. Here, the */
	/* Peripheral Clock is divided by 8 ( gives 250 KSPS with 2Mhz clock ) */
	ADCA.PRESCALER |= ADC_PRESCALER_DIV512_gc;
	
	/* The used Virtual Channel (CH0) must be set in the correct mode */
	/* In this task we will use single ended input, so this mode is selected */
	/* Setup channel 0 to have single ended input and 1x gain */
	ADCA.CH0.CTRL |= ADC_CH_INPUTMODE_SINGLEENDED_gc | ADC_CH_GAIN_1X_gc;
	/* Setting up the which pins to convert. */
	ADCA.CH0.MUXCTRL |= ADC_CH_MUXPOS_PIN6_gc;
	/* Before the ADC can be used it must be enabled */
	ADCA.CTRLA = ADC_ENABLE_bm;
	

	// ADC interrupt level set to LOW
	ADCA.CH0.INTCTRL |= ADC_CH_INTLVL_LO_gc;	// LOW level ADC interrupt on complete.
		
}


/******************************************************************/
/******************************************************************
 Timer initialization                                                                                                
 ******************************************************************/
/******************************************************************/ 

void init_TIMER(void)
{
	TCC1.CTRLA |= TC_CLKSEL_DIV1024_gc;		// 32Mhz clock / 1024 = 31,25 kHz
	TCC1.CTRLB |= TC1_CCAEN_bm;				// CC channel A enabled.
	TCC1.INTCTRLA = 0x02;					// Set MID level overflow/underflow interrupts for clock 1.
	TCC1.PER = 31250*T;						// Set interrupt frequency for the PID control (31,25kHZ*Timestep).	
}


/******************************************************************/
/******************************************************************
 LCD initialization                                                                                                
 ******************************************************************/
/******************************************************************/ 

/* LCD HR 3b */
void init_LCD(void)
{
	PORTE.DIR = 0x30;
	PORTE.OUTSET = 0x30; 	//Power LED On, LCD Backlight On
	c42048a_init(); 		// Initialize the LCD Controller
}


/******************************************************************/
/******************************************************************
 Capacitive button initialization                                                                                                
 ******************************************************************/
/******************************************************************/ 

void init_cBUTTON(void)
{
	PORTE.DIR &= ~(0x0F);			// Ports 0-3 have to be set as inputs.
	PORTE.PIN0CTRL = 0x59;			// Pullup (totem pole), rising edge
	PORTE.PIN1CTRL = 0x59;			// Pullup (totem pole), rising edge
	PORTE.PIN2CTRL = 0x59;			// Pullup (totem pole), rising edge
	PORTE.PIN3CTRL = 0x59;			// Pullup (totem pole), rising edge
}


/******************************************************************/
/******************************************************************
 Interrupt initialization                                                                                                
 ******************************************************************/
/******************************************************************/
 
void init_interrupt(void)
{
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;	// Enable all interrupt levels for use.
	sei();	// Enable global interrupts
}


/******************************************************************/
/******************************************************************
 Main program                                                                                            
 ******************************************************************/
/******************************************************************/ 


void init_LEDS(void)
{
	PORTB.DIR |= 0xF0;
	PORTB.OUT = 0xF0;
};




int main (void) 
{ 
	/* Assign values for the global label struct's members */
	state.T_m = 0;	// Measured temperature
	state.Tref = 1;	// Reference temperature
	state.P = 2;	// P component
	state.I = 3;	// Integral component
	state.D = 4;	// Derivative component

	
    /* Initialize all of the functions, that were defined earlier */
	init_CLOCK();
	init_USART();									
   	init_PWM();
	init_ADC();
    init_TIMER();
	init_LCD();
	init_cBUTTON();
	init_interrupt();
	init_LEDS();

	/* When the code is loaded to the development board and the board starts to function, the first thing it prints on the LCD
	is "T_act". */
	lcd_write_alpha_packet(LABEL_T_ACT);
	
	
   	while(1)	// infinite loop											
   	{ 
	
		if(TCC0.INTFLAGS & 0x01)
		{
			TCC0.INTFLAGS |= 0x01;
			TCC0.CCABUF = pid_value;
		}
	
		/* Sends the Dutycycle and temperature values via UART */
		sendString(dutycycle);
		itoa(pid_value,tx_buffer1,10);			// converts pid_value's integer value to a string and places it to the variable tx_buffer1
		sendString(tx_buffer1);					// sends the integer value via UART
		sendString("\r\n");						// carrier return & row change
	
		sendString(uart_temp);
		itoa(values[state.T_m],tx_buffer2,10);	// converts measured temperature's integer value to a string and places it to the variable tx_buffer2
		sendString(tx_buffer2);					// sends the integer value via UART 
		sendString("\r\n");						// carrier return & row change
	
	
		c42048a_set_numeric_dec(values[machine_state]);	// Print the value of the current state on the LCD
			
	
		/* State machine's user interface. The buttons and their effects are listed below */
		/* The CS0 button's purpose is to change states between the measured temperature, reference temperature and proportional component.
		It also acts as a shortcut from states P, I or D to measured temperature */
	
		if((PORTE.IN & 0x01 ) == 0x01)		// if Button CS0 is pressed 
		{
			if(machine_state == state.T_m) {
				cli();
				machine_state = state.Tref;						// Change the state to reference temperature
				sei();						
				c42048a_set_numeric_dec(values[state.Tref]);	// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_T_REF);			// Print label of current state on LCD
			}
		
			else if(machine_state == state.Tref){
				cli();
				machine_state = state.P;					// Change the state to the P value
				sei();
				c42048a_set_numeric_dec(values[state.P]);	// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_P);			// Print label of current state on LCD
			}
		
			else if(machine_state == state.P){
				cli();
				machine_state = state.T_m;					// Change the state to measured temperature
				sei();					
				c42048a_set_numeric_dec(values[state.T_m]);	// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_T_ACT);		// Print label of current state on LCD
			}
		
			else if(machine_state == state.I){
				cli();
				machine_state = state.T_m;					// Change the state to measured temperature
				sei();
				c42048a_set_numeric_dec(values[state.T_m]);	// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_T_ACT);		// Print label of the current state on LCD
			}
		
			else 
				{
				cli();
				machine_state = state.T_m;					// Change the state to measured temperature
				sei();
				c42048a_set_numeric_dec(values[state.T_m]);	// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_T_ACT);		// Print label of the current state on LCD
			}
		
		}
	
	
		/* ---------------------------------------------------------------------------*/
		/* The CS1 button is used to scroll between the Proportional, Integrative and Derivative -states. It does not change
		the state if the current state is either the measured temp or reference temp. */
	
		if((PORTE.IN & 0x02 ) == 0x02)		// if button CS1 is pressed
		{
			if(machine_state == state.T_m){
				cli();
				machine_state = state.T_m;					// Keep the state the same
				sei();					
				c42048a_set_numeric_dec(values[state.T_m]);	// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_T_ACT);		// Print label of the current state on LCD
			}
		
			else if(machine_state == state.Tref){
				cli();
				machine_state = state.Tref;					// Keep the state the same
				sei();
				c42048a_set_numeric_dec(values[state.Tref]);// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_T_REF);		// Print label of the current state on LCD
			}
		
			else if(machine_state == state.P){
				cli();
				machine_state = state.I;					// Change state to Integral component
				sei();
				c42048a_set_numeric_dec(values[state.I]);	// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_I);			// Print label of the current state on LCD
			}
		
			else if(machine_state == state.I){
				cli();
				machine_state = state.D;					// Change state to derivative component
				sei();
				c42048a_set_numeric_dec(values[state.D]);	// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_D);			// Print label of the current state on LCD
			}
		
			else
				{
				cli();
				machine_state = state.P;					// Change state to proportional component
				sei();
				c42048a_set_numeric_dec(values[state.P]);	// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_P);			// Print label of the current state on LCD
			}
		
		}
	
	
	
		/* ---------------------------------------------------------------------------*/
		/* This part is for the CS2 button. The purpose of the CS2 button is to increase the values of the Reference Temperature,
		Proportional, Integral and Derivative -components' values. However, it should not allow the user to change the value of the actual temperature,
		because the actual temperature is measured by the temperature probe! */
	
		if((PORTE.IN & 0x04 ) == 0x04)		// if button CS2 is pressed
		{
			if(machine_state == state.T_m){
				cli();
				machine_state = state.T_m;							// Keep the state the same. Do not allow the user to increase the value of the measured temperature!
				sei();							
				c42048a_set_numeric_dec(values[state.T_m]);			// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_T_ACT);				// Print label of the current state on LCD
			}
		
			else if(machine_state == state.Tref){
				if(values[state.Tref] >= VALUES_MAX) {				// if the reference temperature is larger or equal to the set max value
					cli();				
					values[state.Tref] = VALUES_MAX;
					sei();
				}
				else {
					cli();
					values[state.Tref] = values[state.Tref] + 1;	// increase the reference temperature by one
					sei();
				}
				c42048a_set_numeric_dec(values[state.Tref]);		// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_T_REF);				// Print label of the current state on LCD
			}	
		
			else if(machine_state == state.P){
				if(values[state.P] >= VALUES_MAX){					// If the proportional component's value is larger or equal to the set max value
					cli();
					values[state.P] = VALUES_MAX;
					sei();
				}
				else {
					cli();
					values[state.P] = values[state.P] + 1;			// increase the proportional component's value by one
					sei();
				}
				c42048a_set_numeric_dec(values[state.P]);			// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_P);					// Print label of the current state on LCD
			}
		
			else if(machine_state == state.I){
				if(values[state.I] >= VALUES_MAX){					// If the value of the integral component is larger or equal to the set max value
					cli();
					values[state.I] = VALUES_MAX;
					sei();
				}
				else{
					cli();
					values[state.I] = values[state.I] + 1;			// increase the integral component's value by one
					sei();
				}
				c42048a_set_numeric_dec(values[state.I]);			// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_I);					// Print label of the current state on LCD
			}
		
			else 
				{
				if(values[state.D] >= VALUES_MAX){					// If the value of the derivative term is larger or equal to the set max value
					cli();
					values[state.D] = VALUES_MAX;
					sei();
				}
				else{
					cli();
					values[state.D] = values[state.D] + 1;			// increase the derivative component's value by one
					sei();
				}
				c42048a_set_numeric_dec(values[state.D]);			// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_D);					// Print label of the current state on LCD
			}	
		
		}
	
	
		/* ----------------------------------------------------------------------------*/
		/* This part is for the CS3 button. The purpose of the CS3 button is to decrease the values of the reference temperature,
		Proportional, Integral and Derivative -components' values. However, it should not allow the user to change the value of the actual temperature,
		because the actual temperature is measured by the temperature probe! */
	
		if((PORTE.IN & 0x08 ) == 0x08)		// if button CS3 is pressed
		{
			if(machine_state == state.T_m){
				cli();
				machine_state = state.T_m;							// Keep the state the same. Don't allow the changing of the value!
				sei();
				c42048a_set_numeric_dec(values[state.T_m]);			// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_T_ACT);				// Print label of the current state on LCD
				}
		
			else if(machine_state == state.Tref){
				if(values[state.Tref] <= VALUES_MIN) {				// if the value of reference temperature is less than or equal to the minimum set value
					cli();
					values[state.Tref] = VALUES_MIN;
					sei();
				}
				else {
					cli();
					values[state.Tref] = values[state.Tref] - 1;	// decrease the value of reference temperature by one
					sei();
				}
				c42048a_set_numeric_dec(values[state.Tref]);		// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_T_REF);				// Print label of the current state on LCD
			}
		
			else if(machine_state == state.P){
				if(values[state.P] <= VALUES_MIN){					// if the value of the proportional component is less than or equal to the minimum set value
					cli();
					values[state.P] = VALUES_MIN;
					sei();
				}
				else {
					cli();
					values[state.P] = values[state.P] - 1;			// decrease the value of the proportional term by one
					sei();
				}
				c42048a_set_numeric_dec(values[state.P]);			// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_P);					// Print label of the current state on LCD
			}
		
			else if(machine_state == state.I){
				if(values[state.I] <= VALUES_MIN){					// if the value of the integral component is less than or equal to the minimum set value
					cli();
					values[state.I] = VALUES_MIN;
					sei();
				}
				else{
					cli();
					values[state.I] = values[state.I] - 1;			// decrease the value of the integral term by one
					sei();
				}
				c42048a_set_numeric_dec(values[state.I]);			// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_I);					// Print label of the current state on LCD
			}
		
			else
				{
				if(values[state.D] <= VALUES_MIN){					// if the value of the derivative component is less than or equal to the minimum set value
					cli();
					values[state.D] = VALUES_MIN;
					sei();
					}
				else{
					cli();
					values[state.D] = values[state.D] - 1;			// decrease the derivative component's value by one
					sei();
					}
				c42048a_set_numeric_dec(values[state.D]);			// Print value of current state on LCD
				lcd_write_alpha_packet(LABEL_D);					// Print label of the current state on LCD
				
			
				}									
			}
			
		_delay_ms(400);
		     
		}
	
	
}

/******************************************************************/
/******************************************************************
 USART RX interrupt                                                                                      
 ******************************************************************/ 
/******************************************************************/

ISR(USARTC0_RXC_vect) 
{

/* LINK: http://digital.ni.com/public.nsf/allkb/6235680C35A22B5986256FE4006DB8E9 */

	/* Setting values for the global state struct's members */
	state.T_m = 0;	// Measured temp
	state.Tref = 1;	// Reference temp
	state.P = 2;	// Proportional component
	state.I = 3;	// Integrative component
	state.D = 4;	// Derivative component

	    
	char RX_buffer = USART.DATA; 			// Read data from the RX (receive) buffer. Values are sent using Numpad /,*,8,9,5,6,2,3.
	
	/*---------------------------------------------------------------*/
	/* The proportional component */
	/*---------------------------------------------------------------*/
	
	/* The value of the proportional component can be increased or decreased by pressing buttons "/" or "*" */
	
	if(RX_buffer == 0x2f)		// If 0x2f = Key "/" is pressed
	{ 
		machine_state = state.P;					// Change the current state to proportional component
		lcd_write_alpha_packet(LABEL_P);			// Print the label of the current state on the LCD
		
		if(values[state.P] >= VALUES_MAX)			// If the value of the proportional component is greater than or equal to the maximum set value
		{
			values[state.P] = VALUES_MAX;
		}
		else
		{
			values[state.P] = values[state.P] + 1;	// increase the value of the proportional component by one
		}
		
	}
	
	/* -----------------------------------------------------------------*/
	
	if(RX_buffer == 0x2a)		// If 0x2a = Key "*" pressed    
	{
		machine_state = state.P;					// Change the current state to the proportional component
		lcd_write_alpha_packet(LABEL_P);			// Print the label of the current state on the LCD
		
		if(values[state.P] <= VALUES_MIN)			// if the value of the proportional term is less than or equal to the minimum set value
		{
			values[state.P] = VALUES_MIN;
		}
		else
		{
			values[state.P] = values[state.P] - 1;	// decrease the value of the proportional term by one
		}			
	}
	
	/* ----------------------------------------------------------------*/
	/* The Integrative component */
	/* ----------------------------------------------------------------*/
	
	/* The value of the integral component can be increased or decreased by pressing the buttons "8" or "9" */
	
	if(RX_buffer == 0x38)		// If 0x38 = Key "8" pressed 
	{
		machine_state = state.I;					// Change the current state to the Integrative component
		lcd_write_alpha_packet(LABEL_I);			// Print the label of the current state on the LCD
		
		if(values[state.I] >= VALUES_MAX)			// if the value of the integral component is greater than or equal to the set maximum value
		{
			values[state.I] = VALUES_MAX;
		}
		else
		{
			values[state.I] = values[state.I] + 1;	// increase the integral component's value by one
		}
	}
	
	/* ----------------------------------------------------------------*/
	
	if(RX_buffer == 0x39)		// If 0x39 = Key "9" pressed       
	{
		machine_state = state.I;					// Change the current state to the Integral component
		lcd_write_alpha_packet(LABEL_I);			// Print the label of the current state on the LCD
		
		if(values[state.I] <= VALUES_MIN)			// if the value of the integral component is less than or equal to the minimum set value
		{
			values[state.I] = VALUES_MIN;
		}
		else
		{
			values[state.I] = values[state.I] - 1;	// decrease the value of the integral component by one
		}
	}
	
	/* ----------------------------------------------------------------*/
	/* The Derivative component */
	/* ----------------------------------------------------------------*/
	
	/* The value of the derivative component can be increased or decreased by using the buttons "5" and "6" */
	
	if(RX_buffer == 0x35)		// If 0x35 = Key "5" pressed
	{
		machine_state = state.D;					// Change the current state to the Derivative component
		lcd_write_alpha_packet(LABEL_D);			// Print the label of the current state on the LCD
		
		if(values[state.D] >= VALUES_MAX)			// if the value of the derivative component is greater than or equal to the maximum set value
		{
			values[state.D] = VALUES_MAX;
		}
		else
		{
			values[state.D] = values[state.D] + 1;	// increase the derivative component's value by one
		}
		
	}
	
	/* ----------------------------------------------------------------*/
	
	if(RX_buffer == 0x36)		// If 0x36 = Key "6" pressed
	{
		machine_state = state.D;					// Change the current state to the derivative component
		lcd_write_alpha_packet(LABEL_D);			// Print the label of the current state on the LCD
		
		if(values[state.D] <= VALUES_MIN)			// if the value of the derivative component is less than or equal to the minimum set value
		{
			values[state.D] = VALUES_MIN;
		}
		else
		{
			values[state.D] = values[state.D] - 1;	// decrease the derivative component's value by one
		}
	}
	
	/* ----------------------------------------------------------------*/
	/* The Reference Temperature */
	/* ----------------------------------------------------------------*/
	
	/* The value of the reference temperature can be either increased or decreased by using the buttons "2" and "3" */
	
	if(RX_buffer == 0x32)		// If 0x32 = Key "2" pressed
	{
		machine_state = state.Tref;							// Change the current state to the reference temperature
		lcd_write_alpha_packet(LABEL_T_REF);				// Print the label of the current state on the LCD
		
		if(values[state.Tref] >= VALUES_MAX)				// if the value of the reference temperature is greater than or equal to the maximum set value
		{
			values[state.Tref] = VALUES_MAX;
		}
		else
		{
			values[state.Tref] = values[state.Tref] + 1;	// increase the value of the reference temperature by one
		}
	}
	
	/* ----------------------------------------------------------------*/
	
	if(RX_buffer == 0x33)		// If 0x33 = Key "3" pressed
	{
		machine_state = state.Tref;							// Change the current state to reference temperature
		lcd_write_alpha_packet(LABEL_T_REF);				// Print the label of the current state on the LCD
		
		if(values[state.Tref] <= VALUES_MIN)				// If the value of the reference temperature is less than or equal to the minimum set value
		{
			values[state.Tref] = VALUES_MIN;
		}
		else
		{
			values[state.Tref] = values[state.Tref] - 1;	// decrease the reference temperature's value by one
		}
		
	}  

	/* ---------------------------------------------------------------*/
	/* The "Back to measured temperature" -button */
	/* ---------------------------------------------------------------*/
	
	if(RX_buffer == 0x30)		// When 0x30 = Key '0' pressed
	{
	/* The point of this last if clause is to allow the user to return the LCD to the state where
		the user can see the actual measured temperature */
		machine_state = state.T_m;
		lcd_write_alpha_packet(LABEL_T_ACT);
	}
	  

}


/*******************************************************************/
/******************************************************************
 Timer overflow interrupt                                                                                              
  ******************************************************************/
/*******************************************************************/ 

ISR(TCC1_OVF_vect)
{
	PidController();						// PWM duty cycle calculated for PID.
	ADCA.CTRLA |= 0x04;						// Start AD conversion on channel 0 (CH0).	
}	


/*******************************************************************/
/******************************************************************
 Interrupt Service Routine for handling the ADC conversion complete interrupt                                                                                             
  ******************************************************************/
/*******************************************************************/ 

ISR(ADCA_CH0_vect)
{

	/* Give the global struct's members the correct values */
	state.T_m = 0;									// Measured temperature
	state.Tref = 1;									// Reference temperature
	state.P = 2;									// Proportional component
	state.I = 3;									// Integral component
	state.D = 4;									// Derivative component

	volatile float V0;								// The output voltage of LM19 temperature sensor
	volatile float Temperature;						// Calculated temperature in celcius
	volatile int16_t ADC_result;					// The result of the ADC after turning the 16-bit value into a 12-bit value
	volatile int16_t ADC_input;						// ADC input value (16-bit)

	float Vref = 2.0625;							// Reference voltage
	int Top = 2047;									// 12-bit max value
	int Gain = 1;
	
	ADC_input = ADCA.CH0.RES;						// Get ADC input value (16-bit)
	ADC_result = (ADC_input & 0x7FF);				// Mask 16-bit ADC value into a 12-bit value

	/* Solve of the value of the temperature sensor's output voltage based on the ADC_result. The equation is from the Development Board's instruction manual, ADC section 
	Page 325. The following substitutions were made: 
	VINP = V0
	VINN = 0
	RES = ADC_result */
	
	V0 = ((ADC_result)*Vref)/(Gain*(Top+1));
	
	
	/* 	Calculating the temperature value in celcius. The equation is from the LM19 temperature sensor's 
	manual, page 6, table 1. The Equation, that was used, is the one with the temperature range of 
	-10 to +65 degrees celcius. */ 
		 
	Temperature = (1000*(V0-1.8641)) / (-11.71);

	values[state.T_m]= Temperature;					// Place the calculated temperature value to the correct values vector's memory slot
	
}


/******************************************************************/
/******************************************************************
 PID-controller                                                                                              
 ******************************************************************/
/******************************************************************/
 
void PidController(void)
{
	// PWM = 0 -> FULLPOWER!
	/* Give the global struct's members the correct values */
	state.T_m = 0;						// Measured temperature
	state.Tref = 1;						// Reference Temperature
	state.P = 2;						// Proportional component
	state.I = 3;						// Integrative component
	state.D = 4;						// Derivative component
	
	
	/* The local variables, that are only needed inside the PID */
	static float error_sum = 0;			// Used for the integration term. The sum of all the errors
	static float previous_error = 0;	// Previous error value. Saved at the end of each iteration
	
	float error;						// The difference of the ref temp and the measured temp
	float difference;					// sensor_value - previous sensor value, divided by the PID timestep. This variable is used for the Derivative term.
	
	/*
	--------------------------------------------------------------------------------------------------------------------
	Remember that this is how the state structure members can be used to access the different memory slots of the values vector.

	   values[state.T_m] = values[0]		
	   values[state.Tref] = values[1]
	   values[state.P] = values[2]
	   values[state.I] = values[3]
	   values[state.D] = values[4]
	
	By using the methods above, we get the following kinds of local, temporary variables to which the PID gain values are substituted     		to.The reason for using local, temporary variables is to avoid the PID controller from being affected by changes in the P, I, D or 		reference temperature values during an iteration of the PID program.		

	The gains are as follows:
	--------------------------------------------------------------------------------------------------------------------
	*/

	int8_t Pgain = values[state.P];				// Proportional
	int8_t Igain = values[state.I];				// Integral
	int8_t Dgain = values[state.D];				// Derivative 
	int8_t Ref_temp = values[state.Tref];		// Ref temperature
	int8_t Measured_temp = values[state.T_m];	// Measured temperature
	
	/* -----------------------------------------------------------------------------------------------------------------*/

	error = Ref_temp - Measured_temp; 			// Reference temperature - measured temperature
	difference = (previous_error - error) / T; 	// slope for the change in error, divided by the PID timestep
	error_sum = error_sum + Igain*(error * T);	// The sum of all errors * Igain.
	

    /* The maximum and minimum values of the error_sum and difference should be limited in order to keep the PID stable */
	/* Keep changing the limit values until the PID's performance is adequate */

	if (error_sum < (-1280))
	{
		error_sum = -1280;
	}
	if(error_sum > (1280))
	{
		error_sum = 1280;
	}

	/* 

	Duty cycle:
	Combining the Pterm, Iterm and Dterm,
	so pid_value = Pterm + Iterm + Dterm, where

	Pterm = Pgain * error
	Iterm = error_sum
	Dterm = Dgain * difference

	*/

	pid_value = (Pgain*error) + error_sum + (Dgain*difference);


	/* In order to prevent the duty cycle value from getting too large or too small, the values in defined in the beginning will be set as maximum and minimum borders */

	if(pid_value < PWM_MIN)
	{
		pid_value = PWM_MIN;
	}
	if(pid_value > PWM_MAX)
	{
		pid_value = PWM_MAX;
	}
	

	/* Saving the current error as the previous error 
	for the next iteration. */

	previous_error = error;

}



/* These functions were copied from Usart_interrupt_2 exercise from Noppa: */

void sendChar(char c)
{
	while( !(USART.STATUS & USART_DREIF_bm) ); // Wait until DATA buffer is empty
	USART.DATA = c;
}

void sendString(char *text)
{
	while(*text)
	{
		sendChar(*text++);
	}
}
