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

#include "macros.h"
#include "lcd.h"
#include "dc.h"
#include "dcc.h"

// ******** Start 100 Hz Timer - Very Accurate Version

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

volatile uint16_t decisecs=0;
volatile uint16_t screenUpdateDecisecs=0;
volatile uint16_t fastDecisecs=0;
volatile uint8_t scaleTenthsAccum = 0;
uint16_t scaleFactor = 10;
volatile uint8_t eventFlags=0;

#define EVENT_TIME_READ_INPUTS      0x01
#define EVENT_TIME_READ_ADCS        0x02
#define EVENT_TIME_ADJUST_SPEED     0x04
#define EVENT_ANALOG_READ_COMPLETE  0x10

uint32_t loopCount = 0;

void blankCursorLine()
{
	lcd_gotoxy(0,2);
	lcd_puts_p(PSTR("                    "));
}

typedef struct
{
	uint8_t clock_A;
	uint8_t clock_B;
	uint8_t debounced_state;
} DebounceState;

void initDebounceState(DebounceState* d, uint8_t initialState)
{
	d->clock_A = d->clock_B = 0;
	d->debounced_state = initialState;
}

uint8_t debounce(uint8_t raw_inputs, DebounceState* d)
{
	uint8_t delta = raw_inputs ^ d->debounced_state;   //Find all of the changes
	uint8_t changes;

	d->clock_A ^= d->clock_B;                     //Increment the counters
	d->clock_B  = ~d->clock_B;

	d->clock_A &= delta;                       //Reset the counters if no changes
	d->clock_B &= delta;                       //were detected.

	changes = ~((~delta) | d->clock_A | d->clock_B);
	d->debounced_state ^= changes;
	return(changes & ~(d->debounced_state));
}

volatile uint16_t adcValue[8];

void initializeADC()
{
	for(uint8_t i=0; i<8; i++)
		adcValue[i] = 0;

	// Setup ADC for bus voltage monitoring
	ADMUX  = 0x44;  // AVCC reference, ADC4 starting channel
	ADCSRA = _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128 prescaler
	ADCSRB = 0x00;  // Free-running mode
	DIDR0  = 0xF0;  // Turn ADC 4-7 into inputs
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);
}

ISR(ADC_vect)
{
	static uint8_t workingChannel = 0;
	static uint16_t accumulator[4] = {0,0,0,0};
	static uint8_t count = 0;
	
	accumulator[workingChannel++] += ADC;

	if (4 == workingChannel)
	{
		count++;
		workingChannel = 0;
	}
	ADMUX = (ADMUX & 0xF0) | ((workingChannel+4) & 0x07);
	
	if (count >= 64)
	{
		for(uint8_t i=0; i<4; i++)
		{
			adcValue[i] = accumulator[i] / 64;
			accumulator[i] = 0;
		}
		count = 0;
		eventFlags |= EVENT_ANALOG_READ_COMPLETE;
	}

	if (0 == (eventFlags & EVENT_ANALOG_READ_COMPLETE))
	{
		// Trigger the next conversion.  Not using auto-trigger so that we can safely change channels
		ADCSRA |= _BV(ADSC);
	}
}


typedef enum
{
	SCREEN_MAIN_DRAW = 0,
	SCREEN_MAIN_REFRESH = 1,
	SCREEN_MAIN_IDLE = 2,
	
	SCREEN_CONF_MENU_DRAW = 10,
	SCREEN_CONF_MENU_IDLE = 11,

	SCREEN_FAST_RESET_DRAW = 14,
	SCREEN_FAST_RESET_IDLE = 15,

	SCREEN_CONF_OUTPUT_SETUP = 100,
	SCREEN_CONF_OUTPUT_DRAW  = 101,
	SCREEN_CONF_OUTPUT_IDLE  = 102,

	SCREEN_CONF_DIAG_SETUP = 250,
	SCREEN_CONF_DIAG_DRAW  = 251,
	SCREEN_CONF_DIAG_IDLE  = 252,
	
	SCREEN_DONT_KNOW = 255

} ScreenState;


typedef struct
{
	const char* configName;
	ScreenState configScreen;
} ConfigurationOption;

const ConfigurationOption configurationOptions[] = 
{
  { "DC/DCC Output ",     SCREEN_CONF_OUTPUT_SETUP },
  { "Diagnostics",        SCREEN_CONF_DIAG_SETUP },  
};

#define NUM_RATIO_OPTIONS  (sizeof(ratioOptions)/sizeof(ConfigurationOption))
#define NUM_CONF_OPTIONS  (sizeof(configurationOptions)/sizeof(ConfigurationOption))

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

	if (++ticks & 0x01)
		eventFlags |= EVENT_TIME_READ_INPUTS;

	if (ticks >= 10)
	{
		ticks = 0;
		decisecs++;
		screenUpdateDecisecs++;
		eventFlags |= EVENT_TIME_READ_ADCS | EVENT_TIME_ADJUST_SPEED;
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
#define SENSOR_INPUT_MASK (_BV(PD7) | _BV(PD6))

#define SOFTKEY_1 0x01
#define SOFTKEY_2 0x02
#define SOFTKEY_3 0x04
#define SOFTKEY_4 0x08
#define FAULT_IN  0x20
#define SENSOR_LEFT  0x40
#define SENSOR_RIGHT 0x80


void initializeSwitches(void)
{
	DDRC &= ~(PANEL_SWITCH_MASK);  // Make panel switches inputs
	PORTC |= (PANEL_SWITCH_MASK);  // Turn on pull-ups
	DDRD &= ~(SENSOR_INPUT_MASK);  // Make sensor input connections inputs
	PORTD |= (SENSOR_INPUT_MASK);  // Turn on pull-ups (already has externals)
	
	DDRD &= ~(_BV(PD2)); // Fault input
	PORTD |= _BV(PD2); // Fault input
}

uint8_t readSwitches()
{
	uint8_t sensors = (PIND & SENSOR_INPUT_MASK);
	uint8_t switches = ((PINC & PANEL_SWITCH_MASK)>>2);
	uint8_t fault = (PIND & _BV(PD2))?0x20:0x00;
	// We want the upper two bits of sensors and the lower four of switches
	// These are active low, so we need to be careful how we put them together
	
	return (fault & 0x20) | (sensors & 0xC0) | (switches & 0x0F);
}

void init(void)
{
	// Kill watchdog
	MCUSR = 0;
	wdt_reset();
	wdt_enable(WDTO_1S);
	eventFlags = 0;
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
	initializeADC();
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
	lcd_puts_p(PSTR(" The Motorman  v1.0 "));
	lcd_gotoxy(0,1);
	lcd_puts_p(PSTR(" Shuttle Controller "));
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

/* Screen Layouts

//  Main Screen
//  00000000001111111111
//  01234567890123456789
// [ADR:xxxx vv.vV a.aaA]
// [SPD:nnn%  RAMP:yy.ys]
// [DIR:fff    SENSE:-/-]
// [LOAD REV  STOP CONF ]
//  1111 2222 3333 4444
// xxxx = Unit number (4 digit, sNNN for short, or *DC* for DC)
// nnn = Speed, 0-100%
// STOP softkey says "RUN" when stopped
// yy.y = Ramp, 0-25.5 seconds
// a.aa = Track amps
// v.v = Track volts
// fff = FWD/REV (direction)
// REV softkey will say "FWD" if direction is reversed
// [ADR:xxxx  **FAULT** ] <-- Top line if driver is faulted
// 

// Configuration Options
// - DC/DCC
// - Loco Config
// - 

//  Loco Configuration Slots Screen (15 slots?)
//  00000000001111111111
//  01234567890123456789
// [> nn A:xxxx nnn/yy.y]
// [> nn A:xxxx nnn/yy.y]
// [> nn A:xxxx         ]
// [ UP  DOWN SLCT BACK ]
// nn = locomotive slot number or DC (slot 0)
// xxxx = address (or *DC* for DC)
// nnn = max speed %
// yy.y = ramp time


//  Loco Configuration Screen 1
//  00000000001111111111
//  01234567890123456789
// [nn  ADDR  MAX  RAMP ]
// [    xxxx  nnn% yy.ys]
// [    ^               ]
// [ +++  >>> NEXT CNCL ]
// nn = locomotive slot number or DC (slot 0)
// xxxx = address (or *DC* for DC)
// nnn = max speed %
// yy.y = ramp time

//  Loco Configuration Screen 2
//  00000000001111111111
//  01234567890123456789
// [nn   FWD FWD REV REV]
// [ Fnn Fnn Fnn Fnn Fnn]
// [     ^              ]
// [ +++  >>> SAVE CNCL ]
// nn = locomotive slot number or DC (slot 0)
// xxxx = address (or *DC* for DC)
// nnn = max speed %
// yy.y = ramp time


//  Diagnostic Screen
//  00000000001111111111
//  01234567890123456789
// [IN:vv.vV ]
// [A:vv.v B:vv.v I:a.aa]
// [S:-/- F:n           ]
// [PHSA PHSB      BACK ]
// F:n - n=Y/N for faulted
*/
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

#define TRACK_STATUS_SENSOR_LEFT     0x80
#define TRACK_STATUS_SENSOR_RIGHT    0x40
#define TRACK_STATUS_FAULTED         0x20
#define TRACK_STATUS_STOPPED         0x01

typedef struct
{
	uint16_t address;
	bool shortDCCAddress;
	uint16_t maxSpeed;
	uint8_t rampRate;
	uint32_t fwdFunctions;
	uint32_t revFunctions;
	uint32_t allFunctions;
} LocoConfig;


#define DIRECTION_FORWARD 0
#define DIRECTION_REVERSE 1

typedef struct
{
	// Loaded from EEP
	bool dcMode;
	uint8_t activeLocoConfig;
	bool stopped;
	
	// Runtime elements
	int16_t speed;
	int16_t requestedSpeed;
	uint8_t direction;
} OpsConfiguration;

void loadOpsConfiguration(OpsConfiguration* opsConfig)
{
	opsConfig->dcMode = false;
	opsConfig->stopped = false;
	
	if(opsConfig->dcMode)
		opsConfig->activeLocoConfig = 0;
	else
		opsConfig->activeLocoConfig = 1; // FIXME!  Load this!
	
	opsConfig->speed = 0;
	opsConfig->requestedSpeed = 2000;
}

void loadLocoConfiguration(uint8_t whichConfig, LocoConfig* locoConfig)
{
	locoConfig->address = 600;
	locoConfig->shortDCCAddress = false;
	
	locoConfig->maxSpeed = 2000;
	locoConfig->rampRate = 20; // 20 seconds

	locoConfig->fwdFunctions = 1; // F0
	locoConfig->revFunctions = 1; // F0
	locoConfig->allFunctions = 0; // Fnone
}

#define ANALOG_CHANNEL_PHASE_A_VOLTS  0
#define ANALOG_CHANNEL_PHASE_B_VOLTS  1
#define ANALOG_CHANNEL_INPUT_VOLTS    2
#define ANALOG_CHANNEL_TRACK_CURRENT  3

typedef enum
{
	STATE_LEARN = 0,
	STATE_FTOR_WAIT,
	STATE_RTOF_WAIT,
	STATE_REVERSE,
	STATE_FORWARD,
	STATE_REVDECEL,
	STATE_FWDDECEL,
} OpState;


int main(void)
{
	uint8_t buttonsPressed=0;
	uint8_t configMenuOption = 0;
	uint16_t kloopsPerSec=0;
	uint8_t trackStatus = 0;
	uint16_t inputVoltage=0, phaseAVoltage=0, phaseBVoltage=0, trackCurrent=0, trackVoltage=0;

	char screenLineBuffer[21];

	uint8_t configSaveU8 = 0;
	OpState opState = STATE_LEARN;
	ScreenState screenState = SCREEN_MAIN_DRAW;
	LocoConfig currentLoco;
	OpsConfiguration opsConfig;
	
	uint8_t endStopDelay = 0;
	
	DebounceState d;
	uint8_t fwdSensorMask = 0, revSensorMask = 0;
	// Application initialization
	init();
	initDebounceState(&d, 0xFF); // Initialize all high since all inputs are active low
	loadOpsConfiguration(&opsConfig);
	loadLocoConfiguration(opsConfig.activeLocoConfig, &currentLoco);
	
	if (opsConfig.dcMode)
		dc_init();
	else
		dcc_init();
	
	drawSplashScreen();
	wdt_reset();
	loopCount = 0;

	opState = STATE_LEARN;
	opsConfig.requestedSpeed = currentLoco.maxSpeed;

	while (1)
	{
		bool updateData = false;
		loopCount++;
		wdt_reset();

		if (eventFlags & EVENT_TIME_READ_INPUTS)
		{
			eventFlags &= ~(EVENT_TIME_READ_INPUTS);
			
			buttonsPressed = debounce(readSwitches(), &d);

			if (d.debounced_state & FAULT_IN)
				trackStatus &= ~TRACK_STATUS_FAULTED;
			else
				trackStatus |= TRACK_STATUS_FAULTED;
				
			
			if (d.debounced_state & SENSOR_LEFT)
				trackStatus &= ~TRACK_STATUS_SENSOR_LEFT;
			else
				trackStatus |= TRACK_STATUS_SENSOR_LEFT;
				
			if (d.debounced_state & SENSOR_RIGHT)
				trackStatus &= ~TRACK_STATUS_SENSOR_RIGHT;
			else
				trackStatus |= TRACK_STATUS_SENSOR_RIGHT;
		}

		switch(opState)
		{
			case STATE_LEARN:
				fwdSensorMask = 0;
				revSensorMask = 0;
				if (opsConfig.speed > 0)
				{
					if (trackStatus & TRACK_STATUS_SENSOR_LEFT)
					{
						fwdSensorMask = TRACK_STATUS_SENSOR_LEFT;
						revSensorMask = TRACK_STATUS_SENSOR_RIGHT;
						opState = STATE_FWDDECEL;
					} else if (trackStatus & TRACK_STATUS_SENSOR_RIGHT) {
						fwdSensorMask = TRACK_STATUS_SENSOR_RIGHT;
						revSensorMask = TRACK_STATUS_SENSOR_LEFT;
						opState = STATE_FWDDECEL;
					}
				} else if (opsConfig.speed < 0) {
					if (trackStatus & TRACK_STATUS_SENSOR_LEFT)
					{
						fwdSensorMask = TRACK_STATUS_SENSOR_RIGHT;
						revSensorMask = TRACK_STATUS_SENSOR_LEFT;
						opState = STATE_REVDECEL;
					} else if (trackStatus & TRACK_STATUS_SENSOR_RIGHT) {
						fwdSensorMask = TRACK_STATUS_SENSOR_LEFT;
						revSensorMask = TRACK_STATUS_SENSOR_RIGHT;
						opState = STATE_REVDECEL;
					}
				}
				break;
				
			case STATE_FTOR_WAIT:
				opsConfig.requestedSpeed = 0;
				if (0 == endStopDelay)    
					opState = STATE_REVERSE;
				break;
			
			case STATE_RTOF_WAIT:
				opsConfig.requestedSpeed = 0;
				if (0 == endStopDelay)    
					opState = STATE_FORWARD;
				break;
			
			case STATE_FORWARD:
				if (trackStatus & fwdSensorMask)
				{
					opsConfig.requestedSpeed = 0;
					opState = STATE_FWDDECEL;
				} else {
					opsConfig.requestedSpeed = currentLoco.maxSpeed;
				}
				break;

			case STATE_REVERSE:
				if (trackStatus & revSensorMask)
				{
					opsConfig.requestedSpeed = 0;
					opState = STATE_REVDECEL;
				} else {
					opsConfig.requestedSpeed = -currentLoco.maxSpeed;
				}
				break;

			case STATE_REVDECEL:
				opsConfig.requestedSpeed = 0;
				if (0 == opsConfig.speed)
				{
					endStopDelay = 10;
					opState = STATE_RTOF_WAIT;
				}
				break;

			case STATE_FWDDECEL:
				opsConfig.requestedSpeed = 0;
				if (0 == opsConfig.speed)
				{
					endStopDelay = 10;
					opState = STATE_FTOR_WAIT;
				}
				break;
				
			default:
				opState = STATE_LEARN;
				break;
		}

		// Now that we've set the requested speed, implement it periodically using the ramp rate

		if (eventFlags & EVENT_TIME_ADJUST_SPEED)
		{
			eventFlags &= ~(EVENT_TIME_ADJUST_SPEED);
			
			if (endStopDelay > 0)
				endStopDelay--;
			
			// Time to re-evaluate speed
			if (opsConfig.stopped)  // If a stop is requested, immediately set speed to 0
				opsConfig.speed = 0;
			else if (opsConfig.requestedSpeed != opsConfig.speed)
			{
				// Ramp rate is the number of deciseconds it should take to go from 0-max
				// What we need is the speed change per decisecond
				uint16_t incrementsPerDecisec = currentLoco.maxSpeed / currentLoco.rampRate;
				
				// Which is less, the current difference or the ramp increment?
				uint16_t increment = min(abs(opsConfig.speed - opsConfig.requestedSpeed), incrementsPerDecisec);
				
				if (opsConfig.requestedSpeed > opsConfig.speed)
					opsConfig.speed += increment;
				else if (opsConfig.requestedSpeed < opsConfig.speed)
					opsConfig.speed -= increment;
			}

			// Set speed / direction
			if (opsConfig.dcMode)
				dc_setSpeedAndDir(abs(opsConfig.speed) / 100, (opsConfig.speed >= 0)?0:1);
			else
			{
				dcc_setSpeedAndDir(currentLoco.address, currentLoco.shortDCCAddress, 
					abs(opsConfig.speed) / 100, (opsConfig.speed >= 0)?0:1,
					currentLoco.allFunctions | ((opsConfig.speed >= 0)?currentLoco.revFunctions:currentLoco.fwdFunctions));
			}
		}


		if ((eventFlags & EVENT_TIME_READ_ADCS) && (eventFlags & EVENT_ANALOG_READ_COMPLETE))
		{
			// Analog readings are done - go get and process the values
			
			// ADC decivolts = adcValue[] * 33 / 1023
			// Real volts to ADC decivolts = ADC * 11
			// adcValue[ANALOG_CHANNEL_INPUT_VOLTS] * 33 * 11 / 1023
			// simplified, that's adc * 11 / 31
			
			// Iin = adcdecivolts / (20 * 0.07 * 10)
			// Iin = (adc * 33) / (1023 * 14)
			// cIin = adc * 100 / 434
			
			inputVoltage = adcValue[ANALOG_CHANNEL_INPUT_VOLTS] * 11 / 31;
			phaseAVoltage = adcValue[ANALOG_CHANNEL_PHASE_A_VOLTS] * 11 / 31;
			phaseBVoltage = adcValue[ANALOG_CHANNEL_PHASE_B_VOLTS] * 11 / 31;
			trackCurrent = adcValue[ANALOG_CHANNEL_TRACK_CURRENT] * 100 / 434;
			if (opsConfig.dcMode)
				trackVoltage = max(phaseAVoltage, phaseBVoltage);
			else
				trackVoltage = (phaseAVoltage + phaseBVoltage);
			
			// Restart ADC for another round
			eventFlags &= ~(EVENT_ANALOG_READ_COMPLETE | EVENT_TIME_READ_ADCS);
			ADCSRA |= _BV(ADSC);
			updateData = true;
		}
		
		
		dcc_scheduler();
		
		
		switch(screenState)
		{
			case SCREEN_MAIN_DRAW:
				lcd_clrscr();
//  Main Screen
//  00000000001111111111
//  01234567890123456789
// [ADR:xxxx vv.vV a.aaA]
// [SPD:Fnnn  REQ:Fnnn  ]
// [RMP:yy.ys SENSOR:-/-]
// [LOAD REV  STOP CONF ]
				lcd_gotoxy(0, 0);
				lcd_puts_p(PSTR("ADR:"));
				if (opsConfig.dcMode)
					lcd_puts_p(PSTR("*DC*"));
				else
				{
					if (currentLoco.shortDCCAddress)
						snprintf(screenLineBuffer, sizeof(screenLineBuffer), "s%03d", (currentLoco.address & 0xFF));
					else
						snprintf(screenLineBuffer, sizeof(screenLineBuffer), "%04d", currentLoco.address);
					lcd_puts(screenLineBuffer);
				}
				lcd_gotoxy(0,1);
				lcd_puts_p(PSTR("SPD:"));
				lcd_gotoxy(10,1);
				lcd_puts_p(PSTR("REQ:"));

				// Display ramp rate
				lcd_gotoxy(0, 2);
				lcd_puts_p(PSTR("RMP:"));
				snprintf(screenLineBuffer, sizeof(screenLineBuffer), "%02d.%01ds", currentLoco.rampRate/10, currentLoco.rampRate%10);
				lcd_puts(screenLineBuffer);
				
				// Display sensor input status
				lcd_gotoxy(10, 2);
				lcd_puts_p(PSTR("SENSOR:"));
				// Intentional fall-through

			case SCREEN_MAIN_REFRESH:
				lcd_gotoxy(9, 0);
				if (trackStatus & TRACK_STATUS_FAULTED)
				{
					lcd_puts_p(PSTR("** FAULT **"));
				} else {
					snprintf(screenLineBuffer, sizeof(screenLineBuffer), "%02d.%01dV %01d.%02dA", trackVoltage/10, trackVoltage%10, trackCurrent/100, trackCurrent%100);
					lcd_puts(screenLineBuffer);
				}
				
				
				// Display speed
				lcd_gotoxy(4, 1);
				snprintf(screenLineBuffer, sizeof(screenLineBuffer), "%c%03d", (opsConfig.speed<0)?'R':'F', abs(opsConfig.speed)/100);
				lcd_puts(screenLineBuffer);
				lcd_gotoxy(14, 1);
				if (!opsConfig.stopped)
				{
					snprintf(screenLineBuffer, sizeof(screenLineBuffer), "%c%03d", (opsConfig.requestedSpeed<0)?'R':'F', abs(opsConfig.requestedSpeed)/100);
					lcd_puts(screenLineBuffer);
				}
				else
					lcd_puts_p(PSTR("STOP"));
					
				lcd_gotoxy(19,1);
				lcd_putc(opState + '0');
				
				lcd_gotoxy(17,2);
				lcd_putc((trackStatus & TRACK_STATUS_SENSOR_LEFT)?'*':'-');
				lcd_putc('/');
				lcd_putc((trackStatus & TRACK_STATUS_SENSOR_RIGHT)?'*':'-');
				
				drawSoftKeys_p(opsConfig.dcMode?PSTR(""):PSTR("LOAD"),  PSTR("F<>R"), (opsConfig.stopped)?PSTR("RUN!"):PSTR("STOP"), PSTR("CONF"));
				buttonsPressed = 0;
				screenState = SCREEN_MAIN_IDLE;
				break;



			case SCREEN_MAIN_IDLE:
				if(updateData)
					screenState = SCREEN_MAIN_REFRESH;
			
				if ((SOFTKEY_1 & buttonsPressed) && !opsConfig.dcMode)
				{
					// The "Load" key only works in DCC mode
					// Do stuff here to change to load configuration screen
				}
				else if (SOFTKEY_2 & buttonsPressed)
				{
					opsConfig.requestedSpeed = -opsConfig.requestedSpeed;
					opState = STATE_LEARN;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					opsConfig.stopped = !opsConfig.stopped;
					if (opsConfig.stopped)
					{
						opsConfig.speed = 0;
						opState = STATE_LEARN; // Put it back in learning mode
					}
					
					screenState = SCREEN_MAIN_REFRESH;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					// Load Configuration menu
					screenState = SCREEN_CONF_MENU_DRAW;
					configMenuOption = 0;
				}

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

			case SCREEN_CONF_OUTPUT_SETUP:
				lcd_clrscr();
				configSaveU8 = (opsConfig.dcMode)?1:0;
				lcd_gotoxy(0,0);
				lcd_puts("DC / DCC Mode");
				drawSoftKeys_p(PSTR(" DC "),  PSTR("DCC"), PSTR("SAVE"), PSTR("CNCL"));
				// Intentional fall-through

			case SCREEN_CONF_OUTPUT_DRAW:
				lcd_gotoxy(0,1);
				lcd_puts_p(PSTR("[ ] DC Trk Output"));
				lcd_gotoxy(0,2);
				lcd_puts_p(PSTR("[ ] DCC Trk Output"));
				lcd_gotoxy(1, (configSaveU8)?1:2);
				lcd_putc('*');
				screenState = SCREEN_CONF_OUTPUT_IDLE;
				break;

			case SCREEN_CONF_OUTPUT_IDLE:
				if (SOFTKEY_1 & buttonsPressed)
				{
					configSaveU8 = 1;
					screenState = SCREEN_CONF_OUTPUT_DRAW;
				}
				else if (SOFTKEY_2 & buttonsPressed)
				{
					configSaveU8 = 0;
					screenState = SCREEN_CONF_OUTPUT_DRAW;
				}
				else if ((SOFTKEY_3 | SOFTKEY_4) & buttonsPressed)
				{
					if (SOFTKEY_3 & buttonsPressed && (opsConfig.dcMode != (bool)configSaveU8))
					{
						opsConfig.dcMode = (bool)configSaveU8;
						if(opsConfig.dcMode)
						{
							dc_init();
						} else {
							dcc_init();
						}
//						storeConfiguration(status);
					}
					lcd_clrscr();
					screenState = SCREEN_CONF_MENU_DRAW;
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


