#ifndef HELPER_FUNCS
#define HELPER_FUNCS

#include "init_funcs.h"

uint32_t generate_random_position(uint32_t min, uint32_t max) {

    uint32_t range = max - min + 1;
    uint32_t value = 0;

    HAL_RNG_GenerateRandomNumber(&hrng, &value);

    return (value % range) + min;
}

int getAverageGyroReading() {

	uint16_t numberOfReadings = 10;
	int sum = 0;
	float xyz[3];

	for(uint8_t i = 0; i < numberOfReadings; ++i) {
		BSP_GYRO_GetXYZ(xyz);
		sum += (int) xyz[1];
	}
	return (sum / numberOfReadings);
}

#endif
