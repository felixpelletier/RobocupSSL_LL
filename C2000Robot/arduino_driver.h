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

// Read command
static const uint16_t ARDUINO_BAT_MONITOR_1  = 0x01u;
static const uint16_t ARDUINO_BAT_MONITOR_2  = 0x02u;
static const uint16_t ARDUINO_RECALL         = 0x10u;

// Writing command
static const uint16_t ARDUINO_GPIO_2         = 0xB2u;
static const uint16_t ARDUINO_GPIO_3         = 0xB3u;
static const uint16_t ARDUINO_GPIO_4         = 0xB4u;
static const uint16_t ARDUINO_GPIO_5         = 0xB5u;
static const uint16_t ARDUINO_GPIO_6         = 0xB6u;
static const uint16_t ARDUINO_GPIO_7         = 0xB7u;
static const uint16_t ARDUINO_GPIO_8         = 0xB8u;
static const uint16_t ARDUINO_GPIO_9         = 0xB9u;

static const uint16_t ARDUINO_LOW            = 0x00u;
static const uint16_t ARDUINO_HIGH           = 0x01u;
static const uint16_t ARDUINO_RECALL_MESSAGE = 0xEEu;
static const uint16_t ARDUINO_WRITE_FLAG 	 = 0x80u;


static const uint16_t cmdActivateDribbler    = 0x22u;
static const uint16_t cmdDeactivateDribbler  = 0x33u;

/*******************************************************************************
 * User Interface Function
 *******************************************************************************/

void arduino_Init(chip_select CePin);
uint16_t arduino_ReadRegister(uint16_t Reg);
void arduino_WriteRegister(uint16_t Reg, uint16_t Value);
void arduino_ActivateDribbler();
void arduino_DeactivateDribbler();
int arduino_Test();


#endif /* ARDUINO_DRIVER_H_ */
