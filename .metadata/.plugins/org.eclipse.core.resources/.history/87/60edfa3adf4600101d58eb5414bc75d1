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


#define Alarm_Signal_Pin GPIO_PIN_15
#define Alarm_Signal_GPIO_Port GPIOA

#define SECOND_PER_SYMBOL 2U
#define TIME_PER_SYMBOL (SECOND_PER_SYMBOL) * (1000UL)






typedef enum {
	ARMED = 0,			//DONE
	ARMED_COUNTDOWN, //DONE
	ALERT_SMS,		//DONE
	DISARMED,		//DONE
	SET_NEW_PIN,	//DONE
	MENAGE_NUMBER,	//DONE
	REPLACE_NUMBER,	//DONE
	SET_ALERT_TIME,	//DONE
} alarm_state;
extern alarm_state state;



void state_machine_run(char input);
void state_machine_init(char data_blob[64]);
void state_machine_disarmed(void);
void state_machine_armed(void);
void state_machine_countdown(void);
void state_machine_alert(void);
void state_machine_set_alert_time(void);
void state_machine_set_new_pin(void);
void state_machine_menage_number(void);
void state_machine_replace_number(void);

#endif
