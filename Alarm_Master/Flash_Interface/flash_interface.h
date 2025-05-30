/*
 * flash_interface.h
 *
 *  Created on: May 13, 2025
 *      Author: axeel
 */

#ifndef FLASH_INTERFACE_H_
#define FLASH_INTERFACE_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

void flash_write_erase_sector7();
void flash_write_multiple_word(uint32_t addr, uint32_t* data, size_t num);
void flash_read_multiple_words(uint32_t addr, uint32_t* data, size_t n);

//void flash_write_word(uint32_t addr, void* data); useless


#endif /* FLASH_INTERFACE_H_ */
