/*
 * arduino_driver.h
 *
 *  Created on: 2013-11-14
 *      Author: Philippe Babin
 */

#ifndef ARDUINO_DRIVER_H_
#define ARDUINO_DRIVER_H_

#include <stdio.h>
#include "Robocup_Define.h"

static const uint16_t ARDUINO_BAT_MONITOR_1  = 0xA1u << 8;
static const uint16_t ARDUINO_BAT_MONITOR_2  = 0xA2u << 8;
static const uint16_t ARDUINO_GPIO_2         = 0xB2u << 8;
static const uint16_t ARDUINO_GPIO_3         = 0xB3u << 8;
static const uint16_t ARDUINO_GPIO_4         = 0xB4u << 8;
static const uint16_t ARDUINO_GPIO_5         = 0xB5u << 8;
static const uint16_t ARDUINO_GPIO_6         = 0xB6u << 8;
static const uint16_t ARDUINO_GPIO_7         = 0xB7u << 8;
static const uint16_t ARDUINO_GPIO_8         = 0xB8u << 8;
static const uint16_t ARDUINO_GPIO_9         = 0xB9u << 8;
static const uint16_t ARDUINO_RECALL         = 0xC0u << 8;

static const uint16_t	ARDUINO_LOW          = 0x00u << 8;
static const uint16_t	ARDUINO_HIGH         = 0x01u << 8;
static const uint16_t ARDUINO_RECALL_MESSAGE = 0xEEu << 8;



/*******************************************************************************
 * User Interface Function
 *******************************************************************************/

void arduino_Init(chip_select CePin);
uint16_t arduino_ReadRegister(uint16_t Reg);
void arduino_WriteRegister(uint16_t Reg, uint16_t Value);
int arduino_Test();


#endif /* ARDUINO_DRIVER_H_ */
