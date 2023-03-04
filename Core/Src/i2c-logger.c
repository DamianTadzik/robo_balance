/*
 * i2c-logger.c
 *
 *  Created on: 3 mar 2023
 *      Author: brzan
 */
#include "i2c-logger.h"
#include "i2c.h"

#define LOGGER_ADDRESS 0x2A << 1
#define PACKET_SIZE 24
#define FLOATS_NUMBER 7

conv_float_char conv;

HAL_StatusTypeDef log_7_floats(float *datas, uint16_t num)
{
	if (num < 1 || num > 7) return HAL_ERROR;
	uint8_t packet[num * 4];
	for (int i = 0; i < num; i++)		// convert and pack data into packet
	{
		conv.float_var = datas[i];

		for (int k = 0; k < 4; k++)
		{
			packet[k + 4 * i] = conv.buff[k];
		}
	}
	return HAL_I2C_Master_Transmit(&hi2c1, LOGGER_ADDRESS, packet, num * 4, HAL_MAX_DELAY);

}
