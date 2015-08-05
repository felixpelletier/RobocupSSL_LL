/*
 * SPI.cpp
 *
 *  Created on: 2013-10-16
 *      Author: Mathieu Garon
 */


#include "SPI.h"

/*------------------------------------------------------------------------------------------------
 * 	 SPI Initialisation
 * 	 - 8 bit
 * 	 - Master
 * 	 - CPOL/CPHA = 0/0
 * 	 - 1Mbaud
 *------------------------------------------------------------------------------------------------*/
void spi_init(CLK_Handle CLK, SPI_Handle SPI)
{
    CLK_enableSpiaClock(CLK);

    // Reset on, rising edge, 16-bit char bits
    SPI_setCharLength(SPI, SPI_CharLength_8_Bits);

    // Enable master mode, normal phase,
    // enable talk, and SPI int disabled.
    SPI_setMode(SPI, SPI_Mode_Master);
    SPI_enableTx(SPI);
    SPI_setClkPolarity (SPI, SPI_ClkPolarity_OutputRisingEdge_InputFallingEdge);
    SPI_setClkPhase (SPI, SPI_ClkPhase_Delayed);
    SPI_setBaudRate(SPI, SPI_BaudRate_1_MBaud);

    // Relinquish SPI from Reset
    SPI_disableLoopBack(SPI);
    SPI_enable(SPI);

    // Set so breakpoints don't disturb xmission
    SPI_setPriority(SPI, SPI_Priority_FreeRun);


    return;
}

/*------------------------------------------------------------------------------------------------
 * 	 FIFO buffers initialisation
 *------------------------------------------------------------------------------------------------*/
void spi_fifo_init(SPI_Handle SPI)
{

    // Initialize SPI FIFO registers
    SPI_enableChannels(SPI);
    SPI_enableFifoEnh(SPI);
    SPI_resetTxFifo(SPI);
    SPI_clearTxFifoInt(SPI);
    SPI_resetRxFifo(SPI);
    SPI_clearRxFifoInt(SPI);
    SPI_setRxFifoIntLevel(SPI, SPI_FifoLevel_4_Words);

    return;
}

/*------------------------------------------------------------------------------------------------
 * 	GPIO init
 * 	-MOSI = pin 16
 * 	-MISO = pin 17
 * 	-CLK = Pin 18
 * 	- the enable pin will be at the application's driver discretion
 *------------------------------------------------------------------------------------------------*/
void spi_gpio_init(GPIO_Handle GPIO)
{

    //Initialize SPI gpio
    GPIO_setPullUp(GPIO, GPIO_Number_16, GPIO_PullUp_Enable);
    GPIO_setPullUp(GPIO, GPIO_Number_17, GPIO_PullUp_Enable);
    GPIO_setPullUp(GPIO, GPIO_Number_18, GPIO_PullUp_Enable);

    GPIO_setQualification(GPIO, GPIO_Number_16, GPIO_Qual_ASync);
    GPIO_setQualification(GPIO, GPIO_Number_17, GPIO_Qual_ASync);
    GPIO_setQualification(GPIO, GPIO_Number_18, GPIO_Qual_ASync);

    GPIO_setMode(GPIO, GPIO_Number_16, GPIO_16_Mode_SPISIMOA);
    GPIO_setMode(GPIO, GPIO_Number_17, GPIO_17_Mode_SPISOMIA);
    GPIO_setMode(GPIO, GPIO_Number_18, GPIO_18_Mode_SPICLKA);

}
