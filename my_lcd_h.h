/*
 * My_LCD.h
 *
 * Created: 3/1/2013 2:49:04 PM
 * Author: Scott_Schmit
 * 
 * This file contains all of the definitions and function
 * prototypes used to configure and operate the LCD in
 * this project.  All of the definitions were taken from
 * libraries found in the Atmel Software Framework.
 */ 

//#include "My_Board.h"

#ifndef MY_LCD_H_
#define MY_LCD_H_

#define CONFIG_XMEGA_128B1_REVA //Comment this out if using a later revision

// Maximum number of common lines.
#define LCD_MAX_NR_OF_COM  4
// Maximum number of segment lines.
#define LCD_MAX_NBR_OF_SEG  40 // This will change to 25 if using Xmega_B3

// 7-segment with 4 commons terminals
#ifdef CONFIG_XMEGA_128B1_REVA
#  define FIRST_7SEG_4C 2
#else
#  define FIRST_7SEG_4C 10
#endif
#define WIDTH_7SEG_4C 5
#define DIR_7SEG_4C   1 // 1 = from right to left

// 14-segment with 4 commons terminals
#define FIRST_14SEG_4C 12
#define WIDTH_14SEG_4C 7
#define DIR_14SEG_4C   0 // 0 = from left to right

// Pixels With Blinking feature
#define ICON_LEVELBAR 0,0
#define ICON_USB      1,0
#define ICON_AVR      2,0
#define ICON_COLON    3,0
#define ICON_WARNING  0,1
#define ICON_IN       1,1
#define ICON_OUT      2,1
#define ICON_ERROR    3,1

 // Pixels With No-Blinking feature
#define ICON_AM         3,30
#define ICON_PM         0,29
#define ICON_DEGREE_C   3,11
#define ICON_DEGREE_F   2,11
#define ICON_VOLT       1,11
#define ICON_MILLI_VOLT 0,11

#define ICON_LEVEL_1    0,13
#define ICON_LEVEL_2    3,14
#define ICON_LEVEL_3    0,17
#define ICON_LEVEL_4    3,18
#define ICON_LEVEL_5    0,21
#define ICON_LEVEL_6    3,22
#define ICON_LEVEL_7    0,25
#define ICON_LEVEL_8    3,26

#define ICON_DOT_1      3,4
#define ICON_DOT_2      3,6
#define ICON_DOT_3      3,8
#define ICON_DOT_4      3,10

#define ICON_MINUS      0,10

// LCD Function Prototypes
void c42048a_init(void);
void lcd_write_alpha_packet(const uint8_t *data);
void lcd_write_num_packet(const uint8_t *data);
void lcd_set_pixel(uint8_t pix_com, uint8_t pix_seg);
void lcd_clear_pixel(uint8_t pix_com, uint8_t pix_seg);
void c42048a_set_numeric_dec(uint16_t val);
uint8_t conv_ascii(uint16_t val);

#endif /* MY_LCD_H_ */