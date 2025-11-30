#pragma once
// 
// The whole thing is based on Danny Chouinard's work, but his original Google site is now gone
// Basically, there's a one byte sequence number at the front end of each copy of the structured data that
//  runs as either 0x00 or 0xFE.  (0xFF is invalid/unwritten, since that's EEP default)
// You start at the beginning of the array, and then go as far as the sequence number matches.  The last one
//  to match is the last one written.  When you wrap over, you reset the sequence number on the first one.
// This is a pure C version based on KrystianD's C++ implementation, found here:
// https://github.com/KrystianD/avr-eeprom-wear-leveling/
//



#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <avr/eeprom.h>

#define SEQUENCE_SZ_BYTES 1

typedef struct
{
    const uint8_t* eepromStart;
    uint16_t eepromSz;
    uint16_t recordSz;
    int16_t index;

} WearLeveledEEPROM;

bool ewlInit(WearLeveledEEPROM* t, const uint8_t* eepromStart, uint16_t eepromSz, uint16_t recordSz);
bool ewlRead(WearLeveledEEPROM* t, const uint8_t* block, uint16_t blockSz);
bool ewlWrite(WearLeveledEEPROM* t, const uint8_t* block, uint16_t blockSz);
