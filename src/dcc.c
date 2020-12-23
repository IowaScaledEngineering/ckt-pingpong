/*************************************************************************
Title:    AVR DCC Driver using Timer 1 for CKT-PINGPONG
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
#include "dcc.h"

volatile DCCPacket nextDCCPacket;  // This is the next packet the DCC ISR will send when ready

uint16_t dcc_currentAddr;
uint8_t dcc_currentSpeed;
uint32_t dcc_currentFuncs;
bool dcc_shortAddr;
uint8_t dcc_sendResetCount;

AccChange accChangeArray[16];
AccChangeQueue accq;


void accPktQueueInitialize(AccChangeQueue* q, AccChange* pktBufferArray, uint8_t pktBufferArraySz)
{
	q->pktBufferArray = pktBufferArray;
	q->pktBufferArraySz = pktBufferArraySz;
	q->headIdx = q->tailIdx = 0;
	q->full = false;
	memset(q->pktBufferArray, 0, pktBufferArraySz * sizeof(AccChange));
}

uint8_t accPktQueueDepth(AccChangeQueue* q)
{
	uint8_t result = 0;
	if(q->full)
		return(q->pktBufferArraySz);

	result = (uint8_t)(q->headIdx - q->tailIdx) % q->pktBufferArraySz;
	return(result);
}

uint8_t accPktQueuePush(uint16_t address, bool state)
{
	AccChangeQueue* q = &accq;
	// If full, bail with a false
	if (q->full)
		return(0);

	q->pktBufferArray[q->headIdx].address = address;
	q->pktBufferArray[q->headIdx].state = state?1:0;
	q->pktBufferArray[q->headIdx].count = 3;

	if( ++q->headIdx >= q->pktBufferArraySz )
		q->headIdx = 0;

	if (q->headIdx == q->tailIdx)
		q->full = true;
	return(1);
}

uint8_t accPktQueuePopDCCPacket(AccChangeQueue* q, uint8_t* dccData, uint8_t dccDataSz)
{
	uint8_t len = 0;
	uint16_t addr = 0;
	memset(dccData, 0, dccDataSz);
	
	if (0 == accPktQueueDepth(q))
		return(0);

	// Form accessory packet here
	addr = q->pktBufferArray[q->tailIdx].address;
	
	uint16_t nineBitAddr = ((addr+3) / 4);
	uint8_t subAddr = (addr - (nineBitAddr*4) + 3);
	
	dccData[0] = 0x80 | (0x3F & (nineBitAddr));
	
	nineBitAddr = (nineBitAddr>>2); // Shift off the six LSBs we just used, and then shift up by four
	nineBitAddr ^= 0x70; // Invert the remaining address bits
	dccData[1] = 0x88 | (0x70 & nineBitAddr);
	dccData[1] |= 0x06 & (subAddr<<1);
	if (q->pktBufferArray[q->tailIdx].state)
		dccData[1] |= 0x01;

	len = 2;

	// If count-- is zero, go ahead and pop it out
	if ((--q->pktBufferArray[q->tailIdx].count) == 0)
	{
		// Pop this one off, we're done with it
		if( ++q->tailIdx >= q->pktBufferArraySz )
			q->tailIdx = 0;
		q->full = false;
	}
	return(len);
}

uint8_t accPktQueueDrop(AccChangeQueue* q)
{
	if (0 == accPktQueueDepth(q))
		return(0);

	if( ++q->tailIdx >= q->pktBufferArraySz )
		q->tailIdx = 0;
	q->full = false;

	return(1);
}


void dcc_init() 
{
	// DCC uses timer 1 in CTC mode, triggering OC1A and OC1B to toggle every
	// half-bit.  
	dcc_reinit();

	PORTD |= _BV(PD4) | _BV(PD5);
	DDRD |= _BV(PD4) | _BV(PD5);
	// Set to output ones by default
	OCR1A = OCR1B = DCC_ONE_HALF_BIT;
	ICR1 = 0xFFFF;

	TCCR1B = 0;
	TCNT1 = 0; // Reset the timer
	TCCR1A = _BV(COM1A0) | _BV(COM1B0); // OC1A and OC1B both toggle
	TCCR1B = _BV(WGM12);
	
	// If the outputs are the same, toggle ones
	while((bool)(PIND & _BV(PD4)) == (bool)(PIND & _BV(PD5)))
		TCCR1C = _BV(FOC1A);
	
	TIMSK1 |= _BV(OCIE1A);
	TCCR1B = _BV(WGM12) | _BV(CS10);
	
	accPktQueueInitialize(&accq, accChangeArray, sizeof(accChangeArray) / sizeof(AccChange));
}

void dcc_stop()
{
	PORTD &= ~(_BV(PD4) | _BV(PD5));  // Both low for driver high impedence
	DDRD |= _BV(PD4) | _BV(PD5);
	
	TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10)); // Stop the timer for the moment
	TCNT1 = 0; // Reset the timer
	TCCR1A &= ~(_BV(COM1A0) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0)); //Release control of OC1A / OC1B
	TIMSK1 &= ~_BV(OCIE1A);
}

uint8_t dcc_setSpeedAndDir(uint16_t addr, bool isShortAddr, uint8_t speed, uint8_t direction, uint32_t functions)
{
	//001CCCCC  0  DDDDDDDD
	//CCCCC = 11111 - speed
	//U0000000 - U=1 forward, 0=rev
	//U0000001 - emergency stopped
	//11 000000-11 100111
	//Addr byte 2
	speed = min(speed, 126);

	if (speed == 0)
		dcc_currentSpeed = 0;
	else if (speed >= 1 && speed < 127)
		dcc_currentSpeed = speed + 1;

	// In DCC, a high bit on the speed is forward
	dcc_currentSpeed |= (direction?0x00:0x80);

	if (0 == addr)
	{
		// Broadcast
		dcc_shortAddr = true;
		dcc_currentAddr = 0;
	}
	else if (isShortAddr)
	{
		// Short address
		dcc_currentAddr = max(1, min(addr, 127));
		dcc_shortAddr = true;
	} else {
		// Long address
		dcc_currentAddr = max(1, min(addr, 9999));
		dcc_shortAddr = false;
	}
	dcc_currentFuncs = functions;
	
	return 0;
}

void dcc_reinit()
{
	dcc_sendResetCount = 100;
	dcc_currentSpeed = 0;
	dcc_currentAddr = 65535;
}

void dcc_scheduler()
{
	static uint8_t dccStator = 0;
	uint8_t i = 0;
	uint8_t len = 0;

	// If there's no space in the "next" buffer yet, just get out
	if (nextDCCPacket.len != 0)
		return;
		
	if (dcc_sendResetCount)
	{
		dcc_sendResetCount--;
		nextDCCPacket.data[0] = 0x00;
		nextDCCPacket.data[1] = 0x00;
		nextDCCPacket.len = 2;
		return;
	}
	
	if (dcc_currentAddr >= 10000)
	{
		// Just send idles
		nextDCCPacket.data[0] = 0xFF;
		nextDCCPacket.data[1] = 0x00;
		nextDCCPacket.len = 2;
		return;
	}

	if (dcc_shortAddr)
	{
		nextDCCPacket.data[len++] = (0x7F & dcc_currentAddr);
	} else {
		nextDCCPacket.data[len++] = 0b11000000 | (0x3F & (dcc_currentAddr>>8));
		nextDCCPacket.data[len++] = dcc_currentAddr & 0xFF;
	}

	switch(dccStator++)
	{
		case 0: // Send speed/dir
			nextDCCPacket.data[len++] = 0b00111111; // Advanced operations instruction, 128 speed step
			nextDCCPacket.data[len++] = dcc_currentSpeed;
			break;

		case 1:
		case 3:
		case 5:
		case 7:
		case 9:
		case 11:
			// If we have a accessory packet, send it here
			len = accPktQueuePopDCCPacket(&accq, (uint8_t*)nextDCCPacket.data, sizeof(nextDCCPacket.data));
			if (0 == len)
			{
				// If we get zero len back, there's no acc packet to send, so just send idle
				nextDCCPacket.data[0] = 0xFF;
				nextDCCPacket.data[1] = 0x00;
				len = 2;
			}
			break;
		
		case 2:
			i = ((dcc_currentFuncs & 0x1E)>>1) | ((dcc_currentFuncs & 0x01)?0x10:0x00);
			nextDCCPacket.data[len++] = 0b10000000 | (0x1F & i); // Function group 1 (F0-F4)
			break;
			
		case 4:
			nextDCCPacket.data[len++] = 0b10110000 | (0x0F & (dcc_currentFuncs>>5)); // Function group 2 (F5-F8)
			break;

		case 6:
			nextDCCPacket.data[len++] = 0b10100000 | (0x0F & (dcc_currentFuncs>>9)); // Function group 3 (F9-F12)
			break;

		case 8:
			nextDCCPacket.data[len++] = 0b11011110;
			nextDCCPacket.data[len++] = 0xFF & (dcc_currentFuncs>>13); // Function group 4 (F13-F20)
			break;

		case 10:
			nextDCCPacket.data[len++] = 0b11011111;
			nextDCCPacket.data[len++] = 0xFF & (dcc_currentFuncs>>20); // Function group 5 (F21-F28)
			break;

		
		default:
			dccStator = 0;
			len = 0;
			break;
	}

	if (len)
		nextDCCPacket.len = len; // Once we set len, need to not touch the buffer until the ISR clears it
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
						currentPacket.data[currentPacket.len] ^= currentPacket.data[i];
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


