/*
 * My_LCD.c
 *
 * Created: 3/1/2013 3:00:30 PM
 * Author: Scott_Schmit
 *
 * This file contains all of the functions required by
 * this project to operate the LCD. The purpose of this  
 * file is to combine all of the functions fragmented 
 * throughout the Atmel Software Framework into one easy 
 * to read file. 
 */ 

#define BL_PIN 0x20;
#define LED3 0x80;

// If user wishes to use Degree C, Degree F, Volt, and/or milliVolt pixels,
// uncomment the corresponding definitions below
//#define USE_ICON_DEGREE_C
//#define USE_ICON_DEGREE_F
//#define USE_ICON_VOLT
//#define USE_ICON_MILLIVOLT

#include <avr/io.h>
#include <avr/interrupt.h>

//#include "My_Board.h"
#include "My_LCD_h.h"

// Subroutine for handling LCD Frame Interrupts
// This is a user configurable interrupt based
// on the frame rate of the LCD. It can be used
// as a miscellaneous timer by the user since it
// occurs at a constant rate.
ISR(LCD_INT_vect)
{
	// Toggle LED3
	//PORTB.OUTTGL = LED3;
}

void lcd_backlight_on(void) {
	PORTE.OUTSET = BL_PIN;
}

void lcd_backlight_off(void) {
	PORTE.OUTCLR = BL_PIN;
}

void c42048a_init(void) {

	//////////////////////////////////////////////////////////////////////
	//PR.PRGEN
	//     7       6       5       4       3       2       1       0
	// |  LCD  |  USB  |   -   |  AES  |   -   |  RTC  | EVSYS |  DMA  |
	//     0       0       0       0       0       0       0       0
	// Ensure LCD peripheral clock is running
	// It was turned off initially in sysclk_init()
	PR.PRGEN &= ~PR_LCD_bm; // ~0x80
	//////////////////////////////////////////////////////////////////////
	

	//////////////////////////////////////////////////////////////////////
	//LCD.CTRLA
	//     7        6       5        4        3        2       1       0
	// | ENABLE | XBIAS | DATCLK | COMSWP | SEGSWP | CLRDT | SEGON | BLANK|
	//     0        0       0        0        0        0       0       0
	// Clear the display memory
	LCD.CTRLA = LCD_CLRDT_bm; //0x04
	//////////////////////////////////////////////////////////////////////
	
	
	//////////////////////////////////////////////////////////////////////
	//LCD.CTRLB
	//     7        6       5       4        3       2       1      0
	// | PRESC |       CLKDIV[2:0]       | LPWAV |   -   |   DUTY[1:0]   |
	//     0        0       0       0        0       0       0      0
	// Frame Rate set to 64 Hz
	// LP Wave Enabled
	// 1/4 Duty, 1/3 Bias, COM[0:3] used
	LCD.CTRLB = LCD_PRESC_bm | LCD_CLKDIV1_bm | LCD_CLKDIV0_bm | LCD_LPWAV_bm; //0xB8
	//////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////
	//LCD.CTRLC
	//     7       6       5       4        3        2       1       0
	// |   -   |   -   |                   PMSK[5:0]                     |
	//     0       0       0       0        0        0       0       0
	// Enable all segment drivers on micro
	LCD.CTRLC = LCD_PMSK_gm; // 0x3F
	//////////////////////////////////////////////////////////////////////
	
	
	//////////////////////////////////////////////////////////////////////
	//LCD.INTCTRL
	//     7        6       5       4        3       2       1      0
	// |                XIME[4:0]                |   -   | FCINTLVL[1:0] |
	//     0        0       0       0        0       0       0      0
	// Set Frame Complete Interrupt
	// Default Waveforms:	Int Period = XIME[4:0] + 1
	// LP Waveform:			Int Period = (XIME[4:0] + 1) * 2
	LCD.INTCTRL = LCD_XIME2_bm | LCD_XIME1_bm | LCD_XIME0_bm | LCD_FCINTLVL_gm; // 0x3B sets interrupt period to 16 frames with high priority
	//////////////////////////////////////////////////////////////////////
	
	
	//////////////////////////////////////////////////////////////////////
	//LCD.CTRLD
	//     7       6       5       4        3        2       1       0
	// |   -   |   -   |   -   |   -   | BLINKEN |   -   | BLINKRATE[1:0] |
	//     0       0       0        0       0        0       0       0
	// Disable hardware blinking
	LCD.CTRLD = ~LCD_BLINKEN_bm; // 0x00
	//////////////////////////////////////////////////////////////////////
	
	
	//////////////////////////////////////////////////////////////////////
	//LCD.CTRLA
	//     7        6       5        4        3        2       1       0
	// | ENABLE | XBIAS | DATCLK | COMSWP | SEGSWP | CLRDT | SEGON | BLANK|
	//     0        0       0        0        0        0       0       0
	// Enable LCD
	// Enable all LCD segments
	LCD.CTRLA |= LCD_ENABLE_bm | LCD_SEGON_bm; // 0x82
	//////////////////////////////////////////////////////////////////////
	
	// set LCD contrast with signed int value between -32 ~ 31
	// -32 corresponds to a segment voltage of approx 2.5V
	//   0 corresponds to a segment voltage of approx 3.0V
	//  31 corresponds to a segment voltage of approx 3.5V
	// Default is 0 (3.0V)
	LCD.CTRLF = 31;
}

// Use this function to write characters to the 14-segment
// portion of the display.
void lcd_write_alpha_packet(const uint8_t *data)
{
	uint8_t width = WIDTH_14SEG_4C;
	uint8_t dir = DIR_14SEG_4C;
	
	LCD.CTRLG = LCD_TDG_14S_4C_gc | FIRST_14SEG_4C;

	#ifdef CONFIG_XMEGA_128B1_REVA
	if (dir) {
		data += (width-1);
	}
	while (width--) {
		if (*data == '\0') {
			break; // Stop on NULL char
		}
		LCD.CTRLH = *data;
		if (dir) {
			data--;
		} else {
			data++;
		}
	}
	#else // XMEGA_B rev. higher than revA
	if (dir != 0) {
		dir = LCD_DEC_bm;
	}
	while (width--) {
		if (*data == '\0') {
			break; // Stop on NULL char
		}
		LCD.CTRLH = dir | (*data++);
	}
	#endif
}

// Use this function to write characters to the 7-segment
// portion of the display.
void lcd_write_num_packet(const uint8_t *data)
{
	uint8_t width = WIDTH_7SEG_4C;
	uint8_t dir = DIR_7SEG_4C;
	
	LCD.CTRLG = LCD_TDG_7S_4C_gc | FIRST_7SEG_4C;

	#ifdef CONFIG_XMEGA_128B1_REVA
	if (dir) {
		data += (width-1);
	}
	while (width--) {
		if (*data == '\0') {
			break; // Stop on NULL char
		}
		LCD.CTRLH = *data;
		if (dir) {
			data--;
		} else {
			data++;
		}
		
		// Degree C, Degree F, Volt, and MilliVolt Icons get cleared
		// every time decoder operates. If any of these icons are to
		// be used, ensure the following definitions are uncommented
		// in My_Board.h
		#ifdef USE_ICON_DEGREE_C
			LCD.DATA16 |= 0x08;
		#endif
		#ifdef USE_ICON_DEGREE_F
			LCD.DATA11 |= 0x08;
		#endif
		#ifdef USE_ICON_VOLT
			LCD.DATA6 |= 0x08;
		#endif
		#ifdef USE_ICON_MILLIVOLT
			LCD.DATA1 |= 0x08;
		#endif
	}	
	#else // XMEGA_B rev. higher than revA
		if (dir != 0) {
			dir = LCD_DEC_bm;
		}
		while (width--) {
			if (*data == '\0') {
				break; // Stop on NULL char
			}
			
			LCD.CTRLH = dir | (*data++);
			// Degree C, Degree F, Volt, and MilliVolt Icons get cleared
			// every time decoder operates. If any of these icons are to
			// be used, ensure the following definitions are uncommented
			// in My_Board.h
			#ifdef USE_ICON_DEGREE_C
				LCD.DATA16 |= 0x08;
			#endif
			#ifdef USE_ICON_DEGREE_F
				LCD.DATA11 |= 0x08;
			#endif
			#ifdef USE_ICON_VOLT
				LCD.DATA6 |= 0x08;
			#endif
			#ifdef USE_ICON_MILLIVOLT
				LCD.DATA1 |= 0x08;
			#endif
		}
	#endif
}

// This function is used to turn on individual icons on the display.
void lcd_set_pixel(uint8_t pix_com, uint8_t pix_seg) {
	if ((pix_com < LCD_MAX_NR_OF_COM) && (pix_seg < LCD_MAX_NBR_OF_SEG)) {
		register8_t *pixreg = (register8_t *)((uint16_t)&LCD.DATA0)
		+ (pix_com * ((LCD_MAX_NBR_OF_SEG + 7) / 8))
		+ (pix_seg / 8);

		*pixreg |= 1 << (pix_seg % 8);
	}
}

// This function is used to turn off individual icons on the display.
void lcd_clear_pixel(uint8_t pix_com, uint8_t pix_seg) {
	if ((pix_com < LCD_MAX_NR_OF_COM) && (pix_seg < LCD_MAX_NBR_OF_SEG)) {
		register8_t *pixreg = (register8_t *)((uint16_t)&LCD.DATA0)
		+ (pix_com * ((LCD_MAX_NBR_OF_SEG + 7) / 8))
		+ (pix_seg / 8);

		*pixreg &= ~(1 << (pix_seg % 8));
	}
}

void c42048a_set_numeric_dec(uint16_t val)
{
	uint8_t lcd_num[5];
	uint16_t tmp;

	// Compute each digit char value
	tmp = val / 10000;
	if (tmp) {
		lcd_num[0] = conv_ascii(tmp);
	} else
	lcd_num[0] = ' ';

	val = val - tmp*10000;
	tmp = val / 1000;
	if (tmp) {
		lcd_num[1] = conv_ascii(tmp);
	} else
	lcd_num[1] = ' ';

	val = val - tmp*1000;
	tmp = val / 100;
	if (tmp || lcd_num[1]!=' ') {
		lcd_num[2] = conv_ascii(tmp);
	} else lcd_num[2] = ' ';
	val = val - tmp*100;

	tmp = val / 10;
	if (tmp  || lcd_num[2]!=' ' || lcd_num[1]!=' ') {
		lcd_num[3] = conv_ascii(tmp);
	} else lcd_num[3] = ' ';
	val = val - tmp*10;
	lcd_num[4] = conv_ascii(val);

	//! Write to character generator
	lcd_write_num_packet((uint8_t const*)&lcd_num);
}

// This function is used to covert the character to its ASCII value.
uint8_t conv_ascii(uint16_t val)
{
	if (val<= 0x09) {
		val = val + 0x30;
	}
	if ((val >= 0x0A) && (val<= 0x0F)) {
		val = val + 0x37;
	}
	return val;
}