#include "eepromWearLevel.h"

#define min(a,b) ((a)<(b)?(a):(b))

static uint8_t ewlReadSequence(WearLeveledEEPROM* t)
{
    return eeprom_read_byte(t->eepromStart);
}

static int16_t ewlFindLastIdx(WearLeveledEEPROM* t)
{
    uint8_t sequence = ewlReadSequence(t);
    int16_t lastIdx = -1;
    bool hasAny = false;

    for (int16_t i = 0; i < t->eepromSz; i += t->recordSz + SEQUENCE_SZ_BYTES) 
    {
        uint8_t curSequence = eeprom_read_byte((uint8_t*)(t->eepromStart + i));
        if (curSequence != 0xff)
            hasAny = true;
        if (curSequence != sequence)
            break;
        lastIdx = i;
    }

    return hasAny ? lastIdx : -1;
}

static int16_t ewlFindNextEmptyIdx(WearLeveledEEPROM* t)
{
    int16_t lastIdx = ewlFindLastIdx(t);

    if (lastIdx == -1)
        return 0;

    int16_t nextIdx = lastIdx + t->recordSz + SEQUENCE_SZ_BYTES;

    if (nextIdx >= t->eepromSz)
        nextIdx = 0;

    return nextIdx;
}


bool ewlInit(WearLeveledEEPROM* t, const uint8_t* eepromStart, uint16_t eepromSz, uint16_t recordSz)
{
    t->eepromStart = eepromStart;
    t->recordSz = recordSz;
    t->eepromSz = eepromSz;
    return true;
}


bool ewlRead(WearLeveledEEPROM* t, const uint8_t* block, uint16_t blockSz)
{
    int16_t lastIdx = ewlFindLastIdx(t);

    if (lastIdx == -1)
        return false;

    eeprom_read_block((uint8_t*)block, (uint8_t*)(t->eepromStart + lastIdx + SEQUENCE_SZ_BYTES), min(t->recordSz, blockSz));
    return true;
}

bool ewlWrite(WearLeveledEEPROM* t, const uint8_t* block, uint16_t blockSz)
{
	uint8_t sequence = ewlReadSequence(t);
	int16_t nextIdx = ewlFindNextEmptyIdx(t);

    if (nextIdx == 0)
        sequence = sequence ? 0 : 0xfe;

    // save all record bytes
    eeprom_update_block(block, (uint8_t*)(t->eepromStart + nextIdx + SEQUENCE_SZ_BYTES), min(t->recordSz, blockSz));
    // commit transaction by saving the sequence byte
    eeprom_update_byte((uint8_t*)(t->eepromStart + nextIdx), sequence);
    return true;
}