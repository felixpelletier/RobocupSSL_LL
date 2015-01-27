/*
 * CS_demux.h
 *
 *  Created on: 2014-01-09
 *      Author: Philippe Babin
 */

#ifndef DEMUX_DRIVER_H_
#define DEMUX_DRIVER_H_

#include <stdio.h>
#include "Robocup_Define.h"

// Enable the inverse output
//#define NO_DEMUX

/*******************************************************************************
 * User Interface Function
 *******************************************************************************/

void demux_Init( GPIO_Number_e a0, GPIO_Number_e a1, GPIO_Number_e a2, chip_select notConnect);
void demux_connect_to( chip_select CS);
void demux_disconnect();


#endif /* DEMUX_DRIVER_H_ */
