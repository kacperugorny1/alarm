/*
 * flash_interface.c
 *
 *  Created on: May 13, 2025
 *      Author: axeel
 */

#include "flash_interface.h"



void flash_write_erase_sector7(){
	while(FLASH->SR & FLASH_SR_BSY) ;
	__disable_irq();

	//UNLOCK FLASH_CR
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;

	//SECTOR 7 ERASE
	FLASH->CR |= FLASH_CR_SER;
	FLASH->CR |= FLASH_CR_SNB_0 | FLASH_CR_SNB_1 | FLASH_CR_SNB_2;
	FLASH->CR |= FLASH_CR_STRT;

	while(FLASH->SR & FLASH_SR_BSY) ;
	//RETURN TO INIT STATE
	FLASH->CR &= ~FLASH_CR_STRT;
	FLASH->CR &= ~(FLASH_CR_SNB_0 | FLASH_CR_SNB_1 | FLASH_CR_SNB_2);
	FLASH->CR &= ~FLASH_CR_SER;
	FLASH->CR |= FLASH_CR_LOCK;

	__enable_irq();
}



void flash_write_multiple_word(uint32_t addr, uint32_t* data, size_t num){
	//CHECK IF BUSY
	while(FLASH->SR & FLASH_SR_BSY) ;
	__disable_irq();

	//UNLOCK FLASH_CR
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;

	//PICK DOUBLE WORD WRITING
	FLASH->CR |= FLASH_CR_PSIZE_1;
	FLASH->CR &= ~FLASH_CR_PSIZE_0;

	//ENABLE PROGRAMMING MODE
	FLASH->CR |= FLASH_CR_PG;

	for(size_t i = 0; i < num; ++i){
		*((uint32_t *)addr + i) = *(data + i);
	}

	//WAIT TILL COMPLETES
	while(FLASH->SR & FLASH_SR_BSY) ;
	//TURN OFF PROGRAMMING MODE
	FLASH->CR &= ~FLASH_CR_PG;
	FLASH->CR &= ~FLASH_CR_PSIZE_1;

	FLASH->CR |= FLASH_CR_LOCK;
	__enable_irq();
}




void flash_read_multiple_words(uint32_t addr, uint32_t* data, size_t n){
	for(size_t i = 0; i < n; ++i){
		*(data + i) = *((uint32_t*)addr + i);
	}
}


// BASICALLY USELESS THING
/*
void flash_write_word(uint32_t addr, void* data){
	while(FLASH->SR & FLASH_SR_BSY) ;
	__disable_irq();

	//UNLOCK FLASH_CR
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;

	//SETUP - SET WORD AND PROG MODE
	FLASH->CR |= FLASH_CR_PSIZE_1;
	FLASH->CR &= ~FLASH_CR_PSIZE_0;
	FLASH->CR |= FLASH_CR_PG;

	*(uint32_t *)addr = *(uint32_t *)data;

	while(FLASH->SR & FLASH_SR_BSY) ;
	//RETURN TO INIT STATE
	FLASH->CR &= ~FLASH_CR_PG;
	FLASH->CR &= ~FLASH_CR_PSIZE_1;
	FLASH->CR |= FLASH_CR_LOCK;
	__enable_irq();

}
*/
