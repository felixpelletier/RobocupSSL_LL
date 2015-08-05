/*
 * Driver_SCI.h
 *
 *  Created on: 2013-10-16
 *      Author: Mathieu Garon
 */

#ifndef USB_DRIVER_H_
#define USB_DRIVER_H_


#include "Robocup_Define_RadioBase.h"

void USB_init(CLK_Handle CLK, SCI_Handle SCI);
void USB_gpioInit(GPIO_Handle GPIO);

#endif /* USB_DRIVER_H_ */
