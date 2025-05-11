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
#define COUNTDOWN_MS (COUNTDOWN_SECOND) * (1000UL)

typedef enum {
	ARMED,			//DONE
	ARMED_COUNTDOWN,
	ALERT_SMS,
	DISARMED,		//DONE
	SET_NEW_PIN,
	MENAGE_NUMBER,
	ADD_NUMBER,
	REMOVE_NUMBER,
	SET_ALERT_TIME
} alarm_state;
extern alarm_state state;



void state_machine_run(char input, bool changed_inp);
void state_machine_init(uint8_t pin_len, char pin[static pin_len]);
void state_machine_disarmed(void);
void state_machine_armed(void);
void state_machine_countdown(void);

#endif
