/*
 * expander.h
 *
 *  Created on: Jun 8, 2025
 *      Author: axeel
 */

#ifndef EXPANDER_H_
#define EXPANDER_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define SLAVE_ADDRESS_MCP 0b0100000 // write last 0, read last 1


void mcp_init (I2C_HandleTypeDef *i2c);

bool read_expander(void);

void mcp_write_reg (uint8_t reg_addr, uint8_t mess);

void mcp_read_reg (uint8_t reg_addr, uint8_t* mess);

#endif /* EXPANDER_H_ */
