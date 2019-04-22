/*
 * gpio.h
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_
#include <stdbool.h>
#include "main.h"
#include <stdlib.h>
#include <stdint.h>
#include "displayls013b7dh03config.h"

void gpioInit();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();
void gpio_set_interrupt();
void gpioint(uint8_t pin);
void gpioEnableDisplay();
void gpioSetDisplayExtcomin(bool high);


#define	PB0_STATE	(uint32_t)(0x05)
#define	PB0_PRESSED	(uint8_t)(0x01)
#define	PB0_RELEASED	(uint8_t)(0x00)

#define	PB1_STATE	(uint32_t)(0x010)
#define	PB1_PRESSED	(uint8_t)(0x02)
#define	PB1_RELEASED	(uint8_t)(0x03)

#endif /* SRC_GPIO_H_ */
