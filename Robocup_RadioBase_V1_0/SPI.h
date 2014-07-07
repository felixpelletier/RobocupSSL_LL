/*
 * SPI.h
 *
 *  Created on: 2013-10-16
 *      Author: Mathieu Garon
 */

#ifndef SPI_H_
#define SPI_H_

#include "Robocup_Define_RadioBase.h"




void spi_init(CLK_Handle CLK, SPI_Handle SPI);
void spi_fifo_init(SPI_Handle SPI);
void spi_gpio_init(GPIO_Handle GPIO);



#endif /* SPI_H_ */
