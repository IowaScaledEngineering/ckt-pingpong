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

#ifndef _DC_H_
#define _DC_H_


#define BASE_DC_PWM_PERIOD 0x0257 // 20kHz at 12MHz, div 1

void dc_init();
void dc_stop();
void dc_setSpeedAndDir(uint8_t speed, uint8_t direction);

#endif

