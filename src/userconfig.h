#ifndef _USERCONFIG_H_
#define _USERCONFIG_H_

#define NUM_LOCO_OPTIONS 15

typedef struct
{
	uint16_t address;
	bool shortDCCAddress;
	uint8_t maxSpeed;
	uint8_t rampRate;
	uint32_t fwdFunctions;
	uint32_t revFunctions;
	uint32_t allFunctions;
} LocoConfig;

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

#define EEP_OPSCONFIG_FLAGS1         0x0000
#define EEP_OPSCONFIG_ACTIVE_LOCO    0x0001

#define OPSCONFIG_FLAGS1_DC_MODE     0x01
#define OPSCONFIG_FLAGS1_INIT_STOP   0x02

void loadOpsConfiguration(OpsConfiguration* opsConfig);
void saveOpsConfiguration(OpsConfiguration* opsConfig);
void firstTimeInitOpsConfiguration();

// Locomotive configurations are stored at address 0x0040
// Each has 32 bytes reserved for it

// Addr   - Purpose
// +0x0000 - Locomotive address number [15:8]
// +0x0001 - Locomotive address number [7:0]
// +0x0002 - Flags #1
//           Bit 0 - DCC Short Address
//           (other bits undefined)
// +0x0003 - Locomotive max speed (0-100)
// +0x0004 - Locomotive ramp time (1-255 decisecs)
// +0x0005 - Always on functions [31:24]
// +0x0006 - Always on functions [23:16]
// +0x0007 - Always on functions [15:8]
// +0x0008 - Always on functions [7:0]
// +0x0009 - Forward on functions [31:24]
// +0x000A - Forward on functions [23:16]
// +0x000B - Forward on functions [15:8]
// +0x000C - Forward on functions [7:0]
// +0x000E - Reverse on functions [31:24]
// +0x000F - Reverse on functions [23:16]
// +0x0010 - Reverse on functions [15:8]
// +0x0011 - Reverse on functions [7:0]

#define EEP_LOCOCONFIG_MEM_START_ADDR   0x0040

#define EEP_LOCOCONFIG_ADDR_H_OFFSET    0x0000
#define EEP_LOCOCONFIG_ADDR_L_OFFSET    0x0001
#define EEP_LOCOCONFIG_FLAGS_OFFSET     0x0002
#define EEP_LOCOCONFIG_MAXSPEED_OFFSET  0x0003
#define EEP_LOCOCONFIG_RAMPRATE_OFFSET  0x0004
#define EEP_LOCOCONFIG_ALLFUNC_OFFSET   0x0005
#define EEP_LOCOCONFIG_FWDFUNC_OFFSET   0x0009
#define EEP_LOCOCONFIG_REVFUNC_OFFSET   0x000E

#define EEP_LOCOCONFIG_BLOCK_SIZE   32

#define LOCOCONFIG_FLAGS1_DCC_SHORTADDR  0x01


uint32_t loadLocoConfigFunctions(uint16_t offset, uint16_t functionStart);
void loadLocoConfiguration(uint8_t whichConfig, LocoConfig* locoConfig);
bool saveLocoConfiguration(uint8_t whichConfig, LocoConfig* locoConfig);
void firstTimeInitLocoConfiguration();
void firstTimeInitConfig();

#endif