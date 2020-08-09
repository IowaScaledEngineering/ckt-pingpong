#include <stdint.h>

#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))

uint16_t deciIncrement(uint16_t currentVal, uint16_t maxVal, uint16_t minVal, uint8_t e )
{
	uint16_t increment = 1;
	uint32_t newVal = 0;
	uint8_t activeDigit = 0, maxActiveDigit = 0;

	// Go ahead and range the current value in case it's outside spec
	newVal = currentVal = max(minVal, min(maxVal, currentVal));

	switch(e)
	{
		case 0:
			break;
		case 1:
			increment = 10;
			break;
		case 2:
			increment = 100;
			break;
		case 3:
			increment = 1000;
			break;
		case 4:
			increment = 10000;
			break;
		default:
			return (uint16_t)newVal;
	}

	activeDigit = (currentVal / increment) % 10;

	if (newVal + increment > maxVal)
	{
		maxActiveDigit = (maxVal / increment) % 10;
		
		// If incrementing is going to exceed maximum value, handle it
		if (activeDigit + 1 > maxActiveDigit)
		{
			// if it's as simple as exceeding the maximum first digit, roll it over
			newVal -= activeDigit * increment;
		} else if (activeDigit + 1 == maxActiveDigit) {
			// At the maximum active digit
			// See if everything below here is below the maximum value
			if ((maxVal % increment) < (currentVal % increment))
				newVal = (maxVal % increment) + ((activeDigit + 1) * increment);
			else 
				newVal += increment;
		} else {
			// Eh?  should never get here
		}

	} else {
		if (activeDigit + 1 > 9)
		{
			newVal -= activeDigit * increment;
		} else {
			newVal += increment;
		}
	}

	return max(minVal, (uint16_t)newVal);
}
