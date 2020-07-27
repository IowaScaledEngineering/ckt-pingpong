/*************************************************************************
Title:    CKT-PINGPONG v1.0
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2020 Nathan D. Holmes (maverick@drgw.net)
     & Michael Petersen (railfan@drgw.net)

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/atomic.h>

#include "lcd.h"

#define min(a,b) ((a<b)?(a):(b))

// ******** Start 100 Hz Timer - Very Accurate Version

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

volatile uint16_t decisecs=0;
volatile uint16_t updateXmitInterval=20;
volatile uint16_t screenUpdateDecisecs=0;
volatile uint16_t fastDecisecs=0;
volatile uint8_t scaleTenthsAccum = 0;
uint16_t scaleFactor = 10;
volatile uint8_t status=0;

uint16_t kelvinTemp = 0;
uint8_t relHumidity = 0;
uint8_t thVoltage = 0;
uint8_t thAlternator = 0;

#define STATUS_READ_INPUTS 0x01
#define STATUS_FAST_ACTIVE 0x02
#define STATUS_FAST_AMPM   0x04
#define STATUS_REAL_AMPM   0x08
#define STATUS_FAST_HOLDING 0x10 // This hold flag indicates we're actually in hold
#define STATUS_FAST_HOLD   0x20  // This flag indicates that we start going into fast in hold
#define STATUS_TEMP_DEG_F  0x40

#define TIME_FLAGS_DISP_FAST       0x01
#define TIME_FLAGS_DISP_FAST_HOLD  0x02
#define TIME_FLAGS_DISP_REAL_AMPM  0x04
#define TIME_FLAGS_DISP_FAST_AMPM  0x08

#define FAST_MODE (status & STATUS_FAST_ACTIVE)
#define FASTHOLD_MODE (status & STATUS_FAST_HOLDING)


#define EE_ADDR_CONF_FLAGS     0x20

#define CONF_FLAG_FAST_AMPM          0x04
#define CONF_FLAG_REAL_AMPM          0x08
#define CONF_FLAG_FAST_HOLD_START    0x20
#define CONF_FLAG_TEMP_DEG_F         0x40


#define EE_ADDR_FAST_START1_H   0x30
#define EE_ADDR_FAST_START1_M   0x31
#define EE_ADDR_FAST_START1_S   0x32
#define EE_ADDR_FAST_START2_H   0x33
#define EE_ADDR_FAST_START2_M   0x34
#define EE_ADDR_FAST_START2_S   0x35
#define EE_ADDR_FAST_START3_H   0x36
#define EE_ADDR_FAST_START3_M   0x37
#define EE_ADDR_FAST_START3_S   0x38

#define EE_ADDR_FAST_RATIO_H   0x3A
#define EE_ADDR_FAST_RATIO_L   0x3B
#define EE_ADDR_TH_SRC_ADDR    0x3C
#define EE_ADDR_TH_TIMEOUT_H   0x3D
#define EE_ADDR_TH_TIMEOUT_L   0x3E


uint32_t loopCount = 0;

void blankCursorLine()
{
	lcd_gotoxy(0,2);
	lcd_puts_p(PSTR("                    "));
}


uint8_t debounce(uint8_t raw_inputs)
{
	static uint8_t clock_A=0, clock_B=0, debounced_state=0;
	uint8_t delta = raw_inputs ^ debounced_state;   //Find all of the changes
	uint8_t changes;

	clock_A ^= clock_B;                     //Increment the counters
	clock_B  = ~clock_B;

	clock_A &= delta;                       //Reset the counters if no changes
	clock_B &= delta;                       //were detected.

	changes = ~((~delta) | clock_A | clock_B);
	debounced_state ^= changes;
	debounced_state &= 0x0F;
	return(changes & ~(debounced_state));
}

typedef enum
{
	SCREEN_MAIN_DRAW = 0,
	SCREEN_MAIN_IDLE = 1,
	SCREEN_MAIN_UPDATE_TIME = 2,

	SCREEN_CONF_MENU_DRAW = 10,
	SCREEN_CONF_MENU_IDLE = 11,

	SCREEN_FAST_RESET_DRAW = 14,
	SCREEN_FAST_RESET_IDLE = 15,

	SCREEN_CONF_R1224_SETUP = 100,
	SCREEN_CONF_R1224_DRAW = 101,
	SCREEN_CONF_R1224_IDLE = 102,

	SCREEN_CONF_F1224_SETUP = 110,
	SCREEN_CONF_F1224_DRAW = 111,
	SCREEN_CONF_F1224_IDLE = 112,

	SCREEN_CONF_FRATIO_SETUP = 115,
	SCREEN_CONF_FRATIO_DRAW = 116,
	SCREEN_CONF_FRATIO_IDLE = 117,
	SCREEN_CONF_FRATIO_CONFIRM = 118,
	
	SCREEN_CONF_RTIME_SETUP = 120,
	SCREEN_CONF_RTIME_DRAW  = 121,
	SCREEN_CONF_RTIME_IDLE  = 122,
	SCREEN_CONF_RTIME_CONFIRM = 123,

	SCREEN_CONF_FSTART1_SETUP = 127,
	SCREEN_CONF_FSTART2_SETUP = 128,
	SCREEN_CONF_FSTART3_SETUP = 129,	
	SCREEN_CONF_FSTART_COMMON_SETUP = 130,
	SCREEN_CONF_FSTART_DRAW  = 131,
	SCREEN_CONF_FSTART_IDLE  = 132,
	SCREEN_CONF_FSTART_CONFIRM = 133,

	SCREEN_CONF_FSHOLD_SETUP = 135,
	SCREEN_CONF_FSHOLD_DRAW  = 136,
	SCREEN_CONF_FSHOLD_IDLE  = 137,
	SCREEN_CONF_FSHOLD_CONFIRM = 138,

	SCREEN_CONF_PKTINT_SETUP = 140,
	SCREEN_CONF_PKTINT_DRAW = 141,
	SCREEN_CONF_PKTINT_IDLE = 142,
	SCREEN_CONF_PKTINT_CONFIRM = 143,
	
	SCREEN_CONF_RDATE_SETUP = 150,
	SCREEN_CONF_RDATE_DRAW  = 151,
	SCREEN_CONF_RDATE_IDLE  = 152,
	SCREEN_CONF_RDATE_CONFIRM = 153,

	SCREEN_CONF_ADDR_SETUP = 160,
	SCREEN_CONF_ADDR_DRAW  = 161,
	SCREEN_CONF_ADDR_IDLE  = 162,
	SCREEN_CONF_ADDR_CONFIRM = 163,	
	
	SCREEN_CONF_THADDR_SETUP = 170,
	SCREEN_CONF_THADDR_DRAW  = 171,
	SCREEN_CONF_THADDR_IDLE  = 172,
	SCREEN_CONF_THADDR_CONFIRM = 173,		
	
	SCREEN_CONF_TEMPU_SETUP = 176,
	SCREEN_CONF_TEMPU_DRAW = 177,
	SCREEN_CONF_TEMPU_IDLE = 178,	

	SCREEN_CONF_THTIMEOUT_SETUP = 180,
	SCREEN_CONF_THTIMEOUT_DRAW  = 181,
	SCREEN_CONF_THTIMEOUT_IDLE  = 182,
	SCREEN_CONF_THTIMEOUT_CONFIRM = 183,	
		
	SCREEN_CONF_DIAG_SETUP = 250,
	SCREEN_CONF_DIAG_DRAW  = 251,
	SCREEN_CONF_DIAG_IDLE  = 252,
	
	SCREEN_DONT_KNOW = 255

} ScreenState;

#define SOFTKEY_1 0x01
#define SOFTKEY_2 0x02
#define SOFTKEY_3 0x04
#define SOFTKEY_4 0x08

typedef struct
{
	const char* configName;
	ScreenState configScreen;
} ConfigurationOption;

const ConfigurationOption configurationOptions[] = 
{
  { "Real 12/24",     SCREEN_CONF_R1224_SETUP },
  { "Real Time     ", SCREEN_CONF_RTIME_SETUP },
  { "Real Date     ", SCREEN_CONF_RDATE_SETUP },  
  { "Fast 12/24",      SCREEN_CONF_F1224_SETUP },
  { "Fast Ratio     ", SCREEN_CONF_FRATIO_SETUP },  
  { "Fast Start Hold", SCREEN_CONF_FSHOLD_SETUP },  
  { "Fast Start Time 1", SCREEN_CONF_FSTART1_SETUP },
  { "Fast Start Time 2", SCREEN_CONF_FSTART2_SETUP },
  { "Fast Start Time 3", SCREEN_CONF_FSTART3_SETUP },    
  { "Time Pkt Interval", SCREEN_CONF_PKTINT_SETUP },
  { "Node Address",   SCREEN_CONF_ADDR_SETUP },
  { "TH Address",     SCREEN_CONF_THADDR_SETUP },
  { "TH Timeout", SCREEN_CONF_THTIMEOUT_SETUP },
  { "Temperature Units", SCREEN_CONF_TEMPU_SETUP },
  { "Diagnostics",    SCREEN_CONF_DIAG_SETUP },  
};

#define NUM_RATIO_OPTIONS  (sizeof(ratioOptions)/sizeof(ConfigurationOption))
#define NUM_CONF_OPTIONS  (sizeof(configurationOptions)/sizeof(ConfigurationOption))

uint16_t one_count=115; //58us
uint16_t zero_high_count=199; //100us
uint16_t zero_low_count=199; //100us

// 0x02B7 - 58uS 1x prescaler
// 0x04AF - 100uS 1x prescaler

#define DCC_ONE_HALF_BIT   0x02B7
#define DCC_ONE_FULL_BIT   (DCC_ONE_HALF_BIT * 2UL)
#define DCC_ZERO_HALF_BIT  0x04AF
#define DCC_ZERO_FULL_BIT  (DCC_ZERO_HALF_BIT * 2UL)



void dcc_init() 
{
	// DCC uses timer 1 in CTC mode, triggering OC1A and OC1B to toggle every
	// half-bit.  
	
	PORTD &= ~(_BV(PD4) | _BV(PD5));  // High impedence on initial power up
	DDRD |= _BV(PD4) | _BV(PD5);
	// Set to output ones by default
	OCR1A = OCR1B = DCC_ONE_HALF_BIT;

	TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10)); // Stop the timer for the moment
	TCNT1 = 0; // Reset the timer

	TCCR1A = _BV(COM1A0) | _BV(COM1B0); // OC1A and OC1B both toggle

	// If the outputs are the same, toggle ones
	if ((bool)(PIND & _BV(PD4)) == (bool)(PIND & _BV(PD5)))
		TCCR1C = _BV(FOC1A);

	TCCR1B = _BV(WGM12) | _BV(CS10);
	TIMSK1 |= _BV(OCIE1A);
}

typedef enum
{
	DCC_SEND_PREAMBLE = 0,
	DCC_SEND_START_BIT = 1,
	DCC_SEND_DATA_BYTE = 2,
	DCC_SEND_STOP = 3
	
} DCCXmitState;

#define DCC_PACKET_MAX_BYTES  6
#define DCC_PREAMBLE_BITS     14
typedef struct 
{
	uint8_t data[DCC_PACKET_MAX_BYTES];
	uint8_t len;
} DCCPacket;

volatile DCCPacket nextDCCPacket;  // This is the next packet the DCC ISR will send when ready

typedef struct
{
	
	
}


void dcc_scheduler()
{
	// If there's no space in the "next" buffer yet, just get out
	if (nextDCCPacket.len != 0)
		return;

	// Go through function commands first
	
	// Send any speeds that changed
	
	// Finally refresh speeds that haven't changed

}


ISR(TIMER1_COMPA_vect)
{
	static uint8_t phase = 0; // This indicates whether we're on the first or second half of the bit
	static uint8_t nextBit = 1; // This is the bit to send that the previous ISR call computed
	static DCCXmitState dccState = DCC_SEND_PREAMBLE;
	static uint8_t preambleBitsRemaining = DCC_PREAMBLE_BITS+2; // Has a couple extra so we can skip the stop bits
	static uint8_t dataByte = 0;
	static uint8_t dataBit = 7;
	static DCCPacket currentPacket;
	
	if (++phase & 0x01) // Second half of the bit, do nothing
		return;

	// Set the OCR registers first, so that we don't miss a comparison
	OCR1A = OCR1B = (nextBit)?DCC_ONE_HALF_BIT:DCC_ZERO_HALF_BIT;

	// Now that we've made the next bit the current one, calculate the next
	// bit
	switch(dccState)
	{
		case DCC_SEND_PREAMBLE:
			nextBit = 1;
			if (--preambleBitsRemaining == 0)
			{
				if (0 == nextDCCPacket.len)
				{
					// We have nothing to send ready to go, send an idle packet
					currentPacket.data[0] = 0xFF;
					currentPacket.data[1] = 0x00;
					currentPacket.data[2] = 0xFF;
					currentPacket.len = 3;
				} else {
					// Handle loading up currentPacket
					currentPacket.len = min(nextDCCPacket.len, DCC_PACKET_MAX_BYTES-1);
					memcpy(currentPacket.data, (uint8_t*)nextDCCPacket.data, currentPacket.len);
					nextDCCPacket.len = 0; // Reset the length to indicate the buffer's empty

					// Calculate checksum
					currentPacket.data[currentPacket.len] = currentPacket.data[0];
					for (uint8_t i=1; i<currentPacket.len; i++)
						currentPacket.data[currentPacket.len] ^= currentPacket.data[1];
					currentPacket.len++;
				}

				dataByte = 0;
				preambleBitsRemaining = DCC_PREAMBLE_BITS+2;
				dccState = DCC_SEND_START_BIT;
			}
			break;
			
		case DCC_SEND_START_BIT:
			nextBit = 0;
			dataBit = 7;
			dccState = DCC_SEND_DATA_BYTE;
			break;

		case DCC_SEND_DATA_BYTE:
			nextBit = (currentPacket.data[dataByte] & (1<<dataBit))?1:0;
			
			if (0 == dataBit)
			{
				dataByte++;
				if (dataByte >= currentPacket.len)
					dccState = DCC_SEND_PREAMBLE; // A longer preamble can include the stop bit
				else
					dccState = DCC_SEND_START_BIT;
			} else {
				dataBit--;
			}
			break;

		default:
			// No clue why we're here, send a 1, go back to preamble
			nextBit = 1;
			preambleBitsRemaining = DCC_PREAMBLE_BITS+2;
			dccState = DCC_SEND_PREAMBLE;
			break;
	}
}

#define BASE_DC_PWM_PERIOD 0x0257 // 20kHz at 12MHz, div 1

void dc_init()
{
	// DC Mode uses timer 1 in fast PWM mode
	// ICR1 provides the base frequency - in this case, ~20kHz
	PORTD |= _BV(PD4) | _BV(PD5);  // Set both high - this gives lows on both track outputs
	DDRD |= _BV(PD4) | _BV(PD5);

	TIMSK1 &= ~_BV(OCIE1A); // No interrupts in DC mode

	// DC speed and direction are set by which OCx output drops low
	OCR1A = OCR1B = ICR1 = BASE_DC_PWM_PERIOD;

	TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10)); // Stop the timer for the moment
	TCNT1 = 0; // Reset the timer

	TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11); // Both OC1A and OC2A high until match
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
}

// Speed ranges 0-126 (matches DCC)
// Direction is 0 for forward, 1 for reverse

void dc_setSpeedAndDir(uint8_t speed, uint8_t direction)
{
	uint16_t ocValForSpeed = 0;
	
	speed = min(speed, 126);
	
	ocValForSpeed = (BASE_DC_PWM_PERIOD * (uint32_t)speed) / 126L;
	if (direction)
	{
		OCR1A = BASE_DC_PWM_PERIOD - ocValForSpeed;
		OCR1B = BASE_DC_PWM_PERIOD;
	} else {
		OCR1A = BASE_DC_PWM_PERIOD;
		OCR1B = BASE_DC_PWM_PERIOD - ocValForSpeed;
	}
}


void initialize100HzTimer(void)
{
	// Set up timer 3 for 100Hz interrupts
	TCNT3 = 0;
	OCR3A = 0x0752;
	TCCR3A = 0;
	TCCR3B = _BV(WGM32) | _BV(CS31) | _BV(CS30);
	TCCR3C = 0;
	TIFR3 |= _BV(OCF3A);
	TIMSK3 |= _BV(OCIE3A);
	
	decisecs = 0;
	screenUpdateDecisecs = 0;
}


ISR(TIMER3_COMPA_vect)
{
	static uint8_t ticks = 0;

	if (ticks & 0x01)
		status |= STATUS_READ_INPUTS;

	if (++ticks >= 10)
	{
		ticks = 0;
		if (STATUS_FAST_ACTIVE == (status & (STATUS_FAST_ACTIVE | STATUS_FAST_HOLDING)))
		{
			fastDecisecs += scaleFactor / 10;
			scaleTenthsAccum += scaleFactor % 10;
			if (scaleTenthsAccum > 10)
			{
				fastDecisecs++;
				scaleTenthsAccum -= 10;
			}
		}
		decisecs++;
		screenUpdateDecisecs++;
	}
}

#define LCD_BACKLIGHT_PIN  PB4

void lcd_backlightOn()
{
	DDRB |= _BV(LCD_BACKLIGHT_PIN);
	PORTB |= _BV(LCD_BACKLIGHT_PIN);
}

void lcd_backlightOff()
{
	DDRB |= _BV(LCD_BACKLIGHT_PIN);
	PORTB &= ~_BV(LCD_BACKLIGHT_PIN);
}

#define PANEL_SWITCH_MASK (_BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5))

void initializeSwitches(void)
{
	DDRC &= ~(PANEL_SWITCH_MASK);  // Make inputs
	PORTC |= (PANEL_SWITCH_MASK);  // Turn on pull-ups
}

uint8_t readSwitches()
{
	return (PINC & PANEL_SWITCH_MASK)>>2;
}

void init(void)
{
	// Kill watchdog
	MCUSR = 0;
	wdt_reset();
	wdt_enable(WDTO_1S);

	wdt_reset();

	memset((uint8_t*)nextDCCPacket.data, 0, sizeof(nextDCCPacket.data));
	nextDCCPacket.len = 0;

	initializeSwitches();
	// Set Up LCD Panel
	lcd_backlightOn();
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	lcd_gotoxy(0,0);

	// Set up tick timer
	initialize100HzTimer();

	// Enable interrupts
	sei();
}

void drawSplashScreen()
{
	lcd_setup_bigdigits();
	lcd_gotoxy(0,0);
	//               00000000001111111111
	//               01234567890123456789
	//                        ||
	lcd_gotoxy(0,0);
	lcd_puts_p(PSTR("PingPong  Controller"));
	lcd_gotoxy(0,1);
	lcd_puts_p(PSTR("       v1.0"));
	lcd_gotoxy(0,2);
	lcd_puts_p(PSTR("Iowa Scaled Eng 2020"));
	lcd_gotoxy(0,3);
	lcd_puts_p(PSTR("  www.iascaled.com  "));
	
	for(uint8_t i=0; i<30; i++)
	{
		wdt_reset();
		_delay_ms(100);
	}
	lcd_clrscr();
}

void drawSoftKeys_p(const char* key1Text, const char* key2Text, const char* key3Text, const char* key4Text)
{
	uint8_t i;

	lcd_gotoxy(0,3);
	for(i=0; i<20; i++)
		lcd_putc(' ');

	lcd_gotoxy(0,3);
	lcd_puts_p(key1Text);
	lcd_gotoxy(5,3);
	lcd_puts_p(key2Text);
	lcd_gotoxy(10,3);
	lcd_puts_p(key3Text);
	lcd_gotoxy(15,3);
	lcd_puts_p(key4Text);
}

int main(void)
{
	uint8_t buttonsPressed=0;
	uint8_t configMenuOption = 0;
	uint16_t kloopsPerSec=0;
	uint8_t speedy = 0;
	ScreenState screenState = SCREEN_MAIN_DRAW;
	// Application initialization
	init();
	dcc_init();
	
	drawSplashScreen();
	wdt_reset();
	loopCount = 0;
	kloopsPerSec = 0;

	
	while (1)
	{
		loopCount++;
		wdt_reset();

		if (status & STATUS_READ_INPUTS)
		{
			status &= ~(STATUS_READ_INPUTS);
			buttonsPressed = debounce(readSwitches());
			speedy++;
//			dc_setSpeedAndDir(speedy & 0x7F, speedy & 0x80);
		}

		switch(screenState)
		{
		
			case SCREEN_MAIN_DRAW:
				drawSoftKeys_p(FAST_MODE?PSTR("REAL"):PSTR("FAST"),  FAST_MODE?(FASTHOLD_MODE?PSTR("RUN"):PSTR("HOLD")):PSTR(""), FAST_MODE?PSTR("RST"):PSTR(""), PSTR("CONF"));
				buttonsPressed = 0;
				screenState = SCREEN_MAIN_IDLE;
				break;

			case SCREEN_MAIN_IDLE:
				if (SOFTKEY_1 & buttonsPressed)
				{ } // Yeah, whateer
				// Buttons handled, clear
				buttonsPressed = 0;
				break;

			case SCREEN_CONF_MENU_DRAW:
				lcd_clrscr();
				drawSoftKeys_p((configMenuOption>0)?PSTR(" UP "):PSTR(""),  (configMenuOption < NUM_CONF_OPTIONS-1)?PSTR("DOWN"):PSTR(""), PSTR("SLCT"), PSTR("BACK"));

				{
					uint8_t i, baseOptionCount = (configMenuOption / 3) * 3;
					for(i=0; i<3; i++)
					{
						if (i+baseOptionCount >= NUM_CONF_OPTIONS)
							continue;
						if (i+baseOptionCount == configMenuOption)
						{
							lcd_gotoxy(0, i);
							lcd_putc('>');
						}

						lcd_gotoxy(2, i);
						lcd_puts(configurationOptions[i+baseOptionCount].configName);
					}
				}

				screenState = SCREEN_CONF_MENU_IDLE;
				break;
				
			case SCREEN_CONF_MENU_IDLE:
				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
				{
					if (configMenuOption > 0)
						configMenuOption--;
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if (SOFTKEY_2 & buttonsPressed)
				{
					if (configMenuOption < NUM_CONF_OPTIONS-1)
						configMenuOption++;
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					lcd_clrscr();
					screenState = SCREEN_MAIN_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed && (configurationOptions[configMenuOption].configScreen))
				{
					screenState = configurationOptions[configMenuOption].configScreen;
					lcd_clrscr();
				}
				
				// Buttons handled, clear
				buttonsPressed = 0;
				break;

			default:
				lcd_gotoxy(0,1);
				lcd_puts_p(PSTR("Code off in lalaland"));
				lcd_gotoxy(0,2);
				lcd_puts_p(PSTR("Call Nathan"));

				break;
		}

		if (screenUpdateDecisecs >= 10)
		{
			switch(screenState)
			{
				case SCREEN_MAIN_DRAW:
				default:
					break;
			}
			
			screenUpdateDecisecs -= 10;
			kloopsPerSec = loopCount / 1000;
			loopCount = 0;
		}
	}
}



