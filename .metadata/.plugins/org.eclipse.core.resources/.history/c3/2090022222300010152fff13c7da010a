/*
 * state_machine.h
 *
 *  Created on: May 11, 2025
 *      Author: axeel
 */
#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_
#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "lcd_driver.h"


#define Alarm_Signal_Pin GPIO_PIN_15
#define Alarm_Signal_GPIO_Port GPIOA

#define SECOND_PER_SYMBOL 2U
#define TIME_PER_SYMBOL (SECOND_PER_SYMBOL) * (1000UL)


#define COUNTDOWN_SECOND 120U




typedef enum {
	ARMED,			//DONE //TODO ALERT RF SPI
	ARMED_COUNTDOWN, //DONE
	ALERT_SMS,		//TODO GSM MODULE
	DISARMED,		//DONE
	SET_NEW_PIN,	//TODO FLASH + CODE
	MENAGE_NUMBER,	//TODO FLASH + CODE
	ADD_NUMBER,		//TODO FLASH + CODE
	REMOVE_NUMBER,	//TODO FLASH + CODE
	SET_ALERT_TIME	//TODO FLASH + CODE
} alarm_state;
extern alarm_state state;



void state_machine_run(char input, bool changed_inp);
void state_machine_init(uint8_t pin_len, char pin[static pin_len]); // TODO READ FROM FLASH
void state_machine_disarmed(void);
void state_machine_armed(void);
void state_machine_countdown(void);
void state_machine_alert(void); //TODO GSM NOTIFICATION
void state_machine_set_alert_time(void); //TODO WRITE TO FLASH

#endif
