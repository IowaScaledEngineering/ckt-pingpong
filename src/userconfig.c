#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "userconfig.h"
#include "macros.h"


void loadOpsConfiguration(OpsConfiguration* opsConfig)
{
	uint8_t flags1 = eeprom_read_byte((uint8_t*)EEP_OPSCONFIG_FLAGS1);

	memset(opsConfig, 0, sizeof(OpsConfiguration));

	opsConfig->dcMode = (flags1 & OPSCONFIG_FLAGS1_DC_MODE)?true:false;
	opsConfig->stopped = (flags1 & OPSCONFIG_FLAGS1_INIT_STOP)?true:false;;
	
	if(opsConfig->dcMode)
		opsConfig->activeLocoConfig = 0;
	else
		opsConfig->activeLocoConfig = eeprom_read_byte((uint8_t*)EEP_OPSCONFIG_ACTIVE_LOCO);

}

void saveOpsConfiguration(OpsConfiguration* opsConfig)
{
	uint8_t flags1 = 0;
	// Operational configuration is stored at address 0
	// 64 bytes of EEP are reserved for it

	// Addr   - Purpose
	// 0x0000 - Flags #1
	//           Bit 0 - DC operations mode (1=DC 0=DCC)
	//           Bit 1 - Set stopped on startup

	// 0x0001 - Active DCC Locomotive Configuration # (0-15)
	if (opsConfig->dcMode)
		flags1 |= OPSCONFIG_FLAGS1_DC_MODE;

	eeprom_write_byte((uint8_t*)EEP_OPSCONFIG_FLAGS1, flags1);
	eeprom_write_byte((uint8_t*)EEP_OPSCONFIG_ACTIVE_LOCO, min(opsConfig->activeLocoConfig, NUM_LOCO_OPTIONS));

}

void firstTimeInitOpsConfiguration()
{
	// This should only be called if the application has decided to re-initialize configuration
	eeprom_write_byte((uint8_t*)EEP_OPSCONFIG_FLAGS1, 0);
	eeprom_write_byte((uint8_t*)EEP_OPSCONFIG_ACTIVE_LOCO, 1);
}

uint32_t loadLocoConfigFunctions(uint16_t offset, uint16_t functionStart)
{
	uint32_t funcMask = (uint32_t)eeprom_read_byte((uint8_t*)functionStart + offset)<<24;
	funcMask |= (uint32_t)eeprom_read_byte((uint8_t*)functionStart + offset+1)<<16;
	funcMask |= (uint32_t)eeprom_read_byte((uint8_t*)functionStart + offset+2)<<8;
	funcMask |= (uint32_t)eeprom_read_byte((uint8_t*)functionStart + offset);
	return funcMask;
}

void loadLocoConfiguration(uint8_t whichConfig, LocoConfig* locoConfig)
{
	uint16_t offset = EEP_LOCOCONFIG_MEM_START_ADDR + (uint16_t)EEP_LOCOCONFIG_BLOCK_SIZE * min(whichConfig, NUM_LOCO_OPTIONS);
	uint8_t flags1 = 0;

	memset(locoConfig, 0, sizeof(LocoConfig));

	flags1 = eeprom_read_byte((uint8_t*)EEP_LOCOCONFIG_FLAGS_OFFSET + offset);

	locoConfig->address = ((uint16_t)eeprom_read_byte((uint8_t*)EEP_LOCOCONFIG_ADDR_H_OFFSET + offset)<<8) 
		| eeprom_read_byte((uint8_t*)EEP_LOCOCONFIG_ADDR_L_OFFSET + offset);

	if (flags1 & LOCOCONFIG_FLAGS1_DCC_SHORTADDR)
	{
		locoConfig->shortDCCAddress = true;
		locoConfig->address = min(127, locoConfig->address);
	}
	
	locoConfig->maxSpeed = min(100, eeprom_read_byte((uint8_t*)EEP_LOCOCONFIG_MAXSPEED_OFFSET + offset));
	locoConfig->rampRate = max(1, eeprom_read_byte((uint8_t*)EEP_LOCOCONFIG_RAMPRATE_OFFSET + offset));

	locoConfig->allFunctions = loadLocoConfigFunctions(offset, EEP_LOCOCONFIG_ALLFUNC_OFFSET);
	locoConfig->fwdFunctions = loadLocoConfigFunctions(offset, EEP_LOCOCONFIG_FWDFUNC_OFFSET);
	locoConfig->revFunctions = loadLocoConfigFunctions(offset, EEP_LOCOCONFIG_REVFUNC_OFFSET);
}


bool saveLocoConfiguration(uint8_t whichConfig, LocoConfig* locoConfig)
{
	// Should probably do something here.
	uint8_t flags1 = 0;
	uint16_t addr = 0;
	uint16_t offset = EEP_LOCOCONFIG_MEM_START_ADDR + (uint16_t)EEP_LOCOCONFIG_BLOCK_SIZE * min(whichConfig, NUM_LOCO_OPTIONS);

	if (locoConfig->shortDCCAddress)
	{
		flags1 |= LOCOCONFIG_FLAGS1_DCC_SHORTADDR;
		addr = min(127, locoConfig->address);
	} else {
		addr = min(9999, locoConfig->address);
	}

	eeprom_write_byte((uint8_t*)EEP_LOCOCONFIG_ADDR_H_OFFSET + offset, 0xFF & (addr>>8));
	eeprom_write_byte((uint8_t*)EEP_LOCOCONFIG_ADDR_L_OFFSET + offset, 0xFF & addr);
	eeprom_write_byte((uint8_t*)EEP_LOCOCONFIG_FLAGS_OFFSET + offset, flags1);

	eeprom_write_byte((uint8_t*)EEP_LOCOCONFIG_MAXSPEED_OFFSET + offset, min(locoConfig->maxSpeed, 100));
	eeprom_write_byte((uint8_t*)EEP_LOCOCONFIG_RAMPRATE_OFFSET + offset, max(locoConfig->rampRate, 1));

	eeprom_write_byte((uint8_t*)EEP_LOCOCONFIG_ALLFUNC_OFFSET + offset, 0xFF & (locoConfig->allFunctions>>24));
	eeprom_write_byte((uint8_t*)EEP_LOCOCONFIG_ALLFUNC_OFFSET + offset+1, 0xFF & (locoConfig->allFunctions>>16));
	eeprom_write_byte((uint8_t*)EEP_LOCOCONFIG_ALLFUNC_OFFSET + offset+2, 0xFF & (locoConfig->allFunctions>>8));
	eeprom_write_byte((uint8_t*)EEP_LOCOCONFIG_ALLFUNC_OFFSET + offset+3, 0xFF & (locoConfig->allFunctions));

	eeprom_write_byte((uint8_t*)EEP_LOCOCONFIG_FWDFUNC_OFFSET + offset, 0xFF & (locoConfig->fwdFunctions>>24));
	eeprom_write_byte((uint8_t*)EEP_LOCOCONFIG_FWDFUNC_OFFSET + offset+1, 0xFF & (locoConfig->fwdFunctions>>16));
	eeprom_write_byte((uint8_t*)EEP_LOCOCONFIG_FWDFUNC_OFFSET + offset+2, 0xFF & (locoConfig->fwdFunctions>>8));
	eeprom_write_byte((uint8_t*)EEP_LOCOCONFIG_FWDFUNC_OFFSET + offset+3, 0xFF & (locoConfig->fwdFunctions));

	eeprom_write_byte((uint8_t*)EEP_LOCOCONFIG_REVFUNC_OFFSET + offset, 0xFF & (locoConfig->revFunctions>>24));
	eeprom_write_byte((uint8_t*)EEP_LOCOCONFIG_REVFUNC_OFFSET + offset+1, 0xFF & (locoConfig->revFunctions>>16));
	eeprom_write_byte((uint8_t*)EEP_LOCOCONFIG_REVFUNC_OFFSET + offset+2, 0xFF & (locoConfig->revFunctions>>8));
	eeprom_write_byte((uint8_t*)EEP_LOCOCONFIG_REVFUNC_OFFSET + offset+3, 0xFF & (locoConfig->revFunctions));

	return true;
}

void firstTimeInitLocoConfiguration()
{
	// This should only be called if the application has decided to re-initialize configuration
	LocoConfig l;

	for(uint8_t i=0; i<NUM_LOCO_OPTIONS; i++)
	{
		memset(&l, 0, sizeof(LocoConfig));
		if (0 == i)
		{
			// DC configuration
			l.address = 0; // Default DCC address of 3
			l.shortDCCAddress = false;
			l.maxSpeed = 20;
			l.rampRate = 10;
		} else if (1 == i) {
			// Default DCC configuration
			l.address = 0; // Universal "talk to everything" dcc address
			l.shortDCCAddress = true;
			l.maxSpeed = 20;
			l.rampRate = 10;
			l.allFunctions = 0x01; //F0
			l.fwdFunctions = 0;
			l.revFunctions = 0;
		} else {
			l.address = 3; // Default DCC address of 3
			l.shortDCCAddress = true;
			l.maxSpeed = 20;
			l.rampRate = 10;
			l.allFunctions = 0x01; //F0
			l.fwdFunctions = 0;
			l.revFunctions = 0;
		}
		saveLocoConfiguration(i, &l);
	}
}

void firstTimeInitConfig()
{
	firstTimeInitOpsConfiguration();
	firstTimeInitLocoConfiguration();
}
