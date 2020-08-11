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
#ifndef _DCC_H_
#define _DCC_H_

// 0x02B7 - 58uS 1x prescaler
// 0x04AF - 100uS 1x prescaler

#define DCC_ONE_HALF_BIT   0x02B7
#define DCC_ONE_FULL_BIT   (DCC_ONE_HALF_BIT * 2UL)
#define DCC_ZERO_HALF_BIT  0x04AF
#define DCC_ZERO_FULL_BIT  (DCC_ZERO_HALF_BIT * 2UL)
#define DCC_PACKET_MAX_BYTES  6
#define DCC_PREAMBLE_BITS     14

typedef enum
{
	DCC_SEND_PREAMBLE = 0,
	DCC_SEND_START_BIT = 1,
	DCC_SEND_DATA_BYTE = 2,
	DCC_SEND_STOP = 3
	
} DCCXmitState;

typedef struct 
{
	uint8_t data[DCC_PACKET_MAX_BYTES];
	uint8_t len;
} DCCPacket;

typedef struct 
{
	unsigned int address :  11;
	unsigned int state :  1;
	unsigned int count : 2;
} AccChange;

typedef struct
{
	uint8_t headIdx;
	uint8_t tailIdx;
	bool full;
	AccChange* pktBufferArray;
	uint8_t pktBufferArraySz;
} AccChangeQueue;

void dcc_init();
void dcc_stop();
void dcc_reinit();
extern volatile DCCPacket nextDCCPacket;  // This is the next packet the DCC ISR will send when ready
uint8_t dcc_setSpeedAndDir(uint16_t addr, bool isShortAddr, uint8_t speed, uint8_t direction, uint32_t functions);
void dcc_scheduler();
uint8_t accPktQueuePush(uint16_t address, bool state);

#endif
