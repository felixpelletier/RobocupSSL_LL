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

#define	ARDUINO_BAT_MONITOR_1              0xA1 << 8
#define	ARDUINO_BAT_MONITOR_2              0xA2 << 8
#define	ARDUINO_GPIO_2              	   0xB2 << 8
#define	ARDUINO_GPIO_3              	   0xB3 << 8
#define	ARDUINO_GPIO_4              	   0xB4 << 8
#define	ARDUINO_GPIO_5              	   0xB5 << 8
#define	ARDUINO_GPIO_6              	   0xB6 << 8
#define	ARDUINO_GPIO_7              	   0xB7 << 8
#define	ARDUINO_GPIO_8              	   0xB8 << 8
#define	ARDUINO_GPIO_9              	   0xB9 << 8
#define	ARDUINO_RECALL              	   0xC0 << 8

#define	ARDUINO_LOW              	       0x00 << 8
#define	ARDUINO_HIGH              	       0x01 << 8
#define	ARDUINO_RECALL_MESSAGE             0xEE << 8



/*******************************************************************************
 * User Interface Function
 *******************************************************************************/

void arduino_Init(chip_select CePin);
uint16_t arduino_ReadRegister(uint16_t Reg);
void arduino_WriteRegister(uint16_t Reg, uint16_t Value);
int arduino_Test();


#endif /* ARDUINO_DRIVER_H_ */
