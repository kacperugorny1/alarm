/*
 * lcd_driver.h
 *
 *  Created on: May 10, 2025
 *      Author: axeel
 */

#ifndef LCD_DRIVER_H_
#define LCD_DRIVER_H_

#include "stm32f4xx_hal.h"

#define SLAVE_ADDRESS_LCD 0x4E // write last 0, read last 1

void lcd_send_cmd (char cmd);
void lcd_send_data (char data);
void lcd_init (I2C_HandleTypeDef* i2c);
void lcd_send_string (char *str);
void lcd_put_cur(int row, int col);
void lcd_clear(void);

#endif /* LCD_DRIVER_H_ */
