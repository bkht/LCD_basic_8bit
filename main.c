#include "stm32f4xx.h"                  // Device header
#include <stdio.h>

// Display used: 
// Newhaven 4x20 character OLED (green)
// NHD-0420DZW-AG5_Character_OLED_Display_Module
// Note, command delays according to datasheet (600uS, 2mS)
// PORTE
// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
// D7 D6 D5 D4 D3 D2 D1 D0 RW RS  E

#define LED_LD8_R (1U<<5)
#define LED_LD4_G (1U<<12)
#define LED_LD3_O (1U<<13)
#define LED_LD5_R (1U<<14)
#define LED_LD6_B (1U<<15)
#define USER_BUTTON (1U<<0)

#define LCKK (1U<<16)

#define HD44780_E  (1U << 5)
#define HD44780_RS (1U << 6)
#define HD44780_RW (1U << 7)
#define HD44780_D0 (1U << 8)
#define HD44780_D1 (1U << 9)
#define HD44780_D2 (1U << 10)
#define HD44780_D3 (1U << 11)
#define HD44780_D4 (1U << 12)
#define HD44780_D5 (1U << 13)
#define HD44780_D6 (1U << 14)
#define HD44780_D7 (1U << 15)

void LCD_Init(void);
void LCD_Cmd(uint8_t command);
void LCD_Data(uint8_t data);
void delay_ms(uint16_t n);
void delay_us(uint32_t n);
void GPIO_Init(void);
void LCD_GotoXY(uint8_t x, uint8_t y);
void LCD_String(char *str);
void LCD_Clear(void);

int main()
{
	uint16_t counter = 0;
	char buffer[10];
	
	LCD_Init();

	GPIOD->BSRR = (LED_LD5_R | LED_LD4_G | LED_LD6_B);
	delay_ms(1000);
	GPIOD->BSRR = ((LED_LD5_R | LED_LD4_G | LED_LD6_B) << 16);

	LCD_GotoXY(0,0);
	LCD_String("Hello");
	
	LCD_GotoXY(2,1);
	LCD_String("AB");

	LCD_GotoXY(4,2);
	LCD_String("CD");
	
	LCD_GotoXY(6,3);
	LCD_String("EF");
	
	while (1) {
		LCD_GotoXY(10,0);
		sprintf(buffer, "%5d", counter);
		LCD_String(buffer);
		counter++;
		delay_ms(1000);
	}
}

void LCD_GotoXY(uint8_t x, uint8_t y)
{
	uint8_t address = 0;
	
	switch (y) {
		case 1:
			address = 0x40 + x;	// Line 1: 0x40, 0x41, 0x42 .. 0x53
			break;
		case 2:
			address = 0x14 + x;	// Line 2: 0x14, 0x15, 0x16 .. 0x27
			break;
		case 3:
			address = 0x54 + x;	// Line 3: 0x54, 0x55, 0x56 .. 0x67
			break;
		default:
			address = 0x00 + x;	// Line 0: 0x00, 0x01, 0x02 .. 0x13
	}
	LCD_Cmd(address | 0x80);
}

void LCD_String(char *str)
{
   while(*str) {
     LCD_Data(*str++);
   }
}

void LCD_Clear(void)
{
    // Display clear
	LCD_Cmd(0x01);	// 00000001
}

void LCD_Init(void)
{
	// Wait for power stabilization: >= 1ms
	delay_ms(1);

    // Enable clock to GPIO Port
    // Set GPIO pins as output pins
    // Enable GPIO pins
	GPIO_Init();
	
    // Function set command - 0x38 = 8-bit, 2 lines, 5x7 font
//	LCD_Cmd(0x38);	// 00111000
	LCD_Cmd(0x38);	// 00111000

    // Display off
	LCD_Cmd(0x08);	// 00001000
	
    // Display clear
	LCD_Cmd(0x01);	// 00000001
	
    // Function mode set command - 0x06 = increment cursor automatically
	LCD_Cmd(0x06);	// 00000110
	
	LCD_Cmd(0x02);	// 00000010 Home Command

	// Display control - 0x0F = turn on display, cursor blinking
	LCD_Cmd(0x0F);	// 00001111
	
	LCD_Cmd(0x80);	// 10000000 Set DDRAM address to 0
}

void LCD_Cmd(uint8_t command) {

    // Select Command Register - RS = 0
	GPIOE->BSRR = ((HD44780_RS | HD44780_RW | HD44780_E) << 16); // RS=0, RW=0, E=0

    // Write command
	// Set up command on GPIOE8-15
	GPIOE->ODR &= 0x00ff;
	GPIOE->ODR |= (command << 8);
	
    // Secure command - E = 1 for a brief moment (20uS)
	GPIOE->BSRR = HD44780_E; // E=1, to secure command
	delay_us(2);
	GPIOE->BSRR = (HD44780_E << 16); // E=0

    // Delay - Allow LCD to catch-up with MCU
	if (command <= 1) {
		delay_ms(2);
	} else {
		delay_us(600);
	}
}

void LCD_Data(uint8_t data) {
    // Select Data Register - RS = 1
	GPIOE->BSRR = ((HD44780_RW | HD44780_E) << 16); // RW=0 and E=0
	GPIOE->BSRR = HD44780_RS; // RS=1
	
    // Write data
	// Set up data on GPIOE8-15
	GPIOE->ODR &= 0x00ff;
	GPIOE->ODR |= (data << 8);

    // Secure data - E = 1 for a brief moment (20uS)
	GPIOE->BSRR = HD44780_E; // set E=1 to secure command
	delay_us(2);
	GPIOE->BSRR = (HD44780_E << 16); // reset E=0
	
    // Set back to command register - RS = 0
	GPIOE->BSRR = (HD44780_RS << 16); // reset RS=0
	
    // Delay - Allow LCD to catch-up with MCU
	delay_us(2);
}

void delay_ms(uint16_t duration) {
	while(duration-- > 0) {
		delay_us(1000);
	}
}

void delay_us(uint32_t duration) {
	duration *= 3;
	while(duration-- > 0) {
	}
}

void GPIO_Init(void) {
    // Enable clock to GPIO Port
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;            // enable clock for GPIOA (button)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;            // enable clock for GPIOD (LEDs)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;            // enable clock for GPIOE (HD44780)

    // Set GPIO pins as output pins
    // Enable GPIO pins GPIOD15,14,13,12: 01 01 01 01 0x55000000
								// 15141312 11100908 07060504 03020100
	GPIOD->MODER = 0x55000000; 	// 01010101 00000000 00000000 00000000
	GPIOD->OSPEEDR = 0;
	GPIOD->OTYPER = 0;
	GPIOD->PUPDR = 0;

	// Now, lock GPIOE configuration
	GPIOD->LCKR = (LED_LD5_R | LED_LD4_G | LED_LD6_B | LED_LD3_O | LED_LD8_R | LCKK);
	GPIOD->LCKR = (LED_LD5_R | LED_LD4_G | LED_LD6_B | LED_LD3_O | LED_LD8_R);
	GPIOD->LCKR = (LED_LD5_R | LED_LD4_G | LED_LD6_B | LED_LD3_O | LED_LD8_R | LCKK);
	uint32_t lockedD = GPIOD->LCKR;
	lockedD = GPIOD->LCKR;	// 2nd read needed, as the LCKK bit is updated after the first read

// Discovery STM32F4
// For the LCD in 8-bit mode, we use PE5-15
// PE3   Free I/O
// PE4   Free I/O
// PE5   Free I/O - HD44780 E
// PE6   Free I/O - HD44780 RS
// PE7   Free I/O - HD44780 RW
// PE8   Free I/O - HD44780 D0
// PE9   Free I/O - HD44780 D1
// PE10  Free I/O - HD44780 D2
// PE11  Free I/O - HD44780 D3
// PE12  Free I/O - HD44780 D4
// PE13  Free I/O - HD44780 D5
// PE14  Free I/O - HD44780 D6
// PE15  Free I/O - HD44780 D7

// PORTE
// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
// D7 D6 D5 D4 D3 D2 D1 D0 RW RS  E
// 01 01 01 01 01 01 01 01 01 01 01 00 00 00 00 00
//   5     5     5     5     5     4     0     0
// 0x55555400
	GPIOE->MODER = 0x55555400; 	// 01010101 01010101 01010100 00000000
	GPIOE->OSPEEDR = 0;
	GPIOE->OTYPER = 0;
	GPIOE->PUPDR = 0;

    // Now, lock GPIOD configuration
	// 0xffe0 11111111 11100000
	GPIOE->LCKR = 0xffe0 | LCKK;
	GPIOE->LCKR = 0xffe0;
	GPIOE->LCKR = 0xffe0 | LCKK;
	uint32_t lockedE = GPIOE->LCKR;
	lockedE = GPIOE->LCKR;	// 2nd read needed, as the LCKK bit is updated after the first read

	// Indicate GPIOs are locked
	if ((lockedD & LCKK) && (lockedE & LCKK)) {
		GPIOD->BSRR = LED_LD3_O;	// Orange LED indicates successful lock
	}
}

