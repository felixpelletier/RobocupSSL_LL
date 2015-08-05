/*
 * SPI.h
 *
 *  Created on: 2013-10-16
 *      Author: Mathieu Garon
 */

#ifndef SPI_H_
#define SPI_H_

#include "Robocup_Define.h"




void spi_init(CLK_Handle CLK, SPI_Handle SPI);
void spi_fifo_init(SPI_Handle SPI);
void spi_gpio_init(GPIO_Handle GPIO);

inline void SPI_write_8bits(SPI_Handle spiHandle,const uint16_t data)
{
    SPI_Obj *spi = (SPI_Obj *)spiHandle;


    // set the bits, shift by 8 because we only want to write 8 bits.
    spi->SPITXBUF = data << 8;

    return;
} // end of SPI_write() function

#endif /* SPI_H_ */
