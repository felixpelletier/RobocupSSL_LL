/*
 * kicker.h
 *
 *  Created on: 2016-02-04
 *      Author: robocup
 */

#ifndef KICKER_H_
#define KICKER_H_


#include <stdio.h>
#include "Robocup_Define.h"

void kicker_init( GPIO_Number_e kick_pin);
void kicker_activate(int duration);
void kicker_update();

#endif /* KICKER_H_ */
