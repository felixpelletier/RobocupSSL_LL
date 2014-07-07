/*
 * Driver_SCI.cpp
 *
 *  Created on: 2013-10-16
 *      Author: Mathieu Garon
 */


#include "Serial.h"

void scia_init(CLK_Handle CLK, SCI_Handle SCI)
{

    CLK_enableSciaClock(CLK);

    // 1 stop bit,  No loopback
    // No parity,8 char bits,
    // async mode, idle-line protocol
    SCI_disableParity(SCI);
    SCI_setNumStopBits(SCI, SCI_NumStopBits_One);
    SCI_setCharLength(SCI, SCI_CharLength_8_Bits);

    SCI_enableTx(SCI);
    SCI_enableRx(SCI);
    //SCI_enableTxInt(mySci);
    //SCI_enableRxInt(mySci);

    SCI_setBaudRate(SCI, SCI_BaudRate_115_2_kBaud);

    SCI_enableFifoEnh(SCI);
    SCI_resetTxFifo(SCI);
    SCI_clearTxFifoInt(SCI);
    SCI_resetChannels(SCI);
    SCI_setTxFifoIntLevel(SCI, SCI_FifoLevel_Empty);

    SCI_resetRxFifo(SCI);
    SCI_clearRxFifoInt(SCI);
    SCI_setRxFifoIntLevel(SCI, SCI_FifoLevel_4_Words);

    SCI_setPriority(SCI, SCI_Priority_FreeRun);

    SCI_enable(SCI);

    return;
}

void scia_gpio_init(GPIO_Handle GPIO)
{

    GPIO_setPullUp(GPIO, GPIO_Number_28, GPIO_PullUp_Enable);
    GPIO_setPullUp(GPIO, GPIO_Number_29, GPIO_PullUp_Disable);
    GPIO_setQualification(GPIO, GPIO_Number_28, GPIO_Qual_ASync);
    GPIO_setMode(GPIO, GPIO_Number_28, GPIO_28_Mode_SCIRXDA);
    GPIO_setMode(GPIO, GPIO_Number_29, GPIO_29_Mode_SCITXDA);
}
