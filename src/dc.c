/*************************************************************************
Title:    AVR DC PWM Driver using Timer 1
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

#include "macros.h"
#include "dc.h"

void dc_init()
{
	// DC Mode uses timer 1 in fast PWM mode
	// ICR1 provides the base frequency - in this case, ~20kHz
	PORTD |= _BV(PD4) | _BV(PD5);  // Set both high - this gives lows on both track outputs
	DDRD |= _BV(PD4) | _BV(PD5);

	TIMSK1 &= ~_BV(OCIE1A); // No interrupts in DC mode

	// DC speed and direction are set by which OCx output drops low
	OCR1A = OCR1B = ICR1 = BASE_DC_PWM_PERIOD;

	TCCR1B = 0; // Stop the timer for the moment
	TCNT1 = 0; // Reset the timer

	TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11); // Both OC1A and OC2A high until match
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
}

void dc_stop()
{
	PORTD &= ~(_BV(PD4) | _BV(PD5));  // Both low for driver high impedence
	DDRD |= _BV(PD4) | _BV(PD5);
	
	TCCR1B = 0; // Stop the timer for the moment
	TCNT1 = 0; // Reset the timer
	TCCR1A = 0; //Release control of OC1A / OC1B
	TIMSK1 &= ~_BV(OCIE1A);
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
