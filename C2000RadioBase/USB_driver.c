/*
 * Driver_SCI.cpp
 *
 *  Created on: 2013-10-16
 *      Author: Mathieu Garon
 */


#include "USB_driver.h"

void USB_init(CLK_Handle CLK, SCI_Handle SCI)
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
    //SCI_enableTxInt(SCI);
    //SCI_enableRxInt(SCI);
    SCI_disableLoopBack(SCI);

    SCI_setBaudRate(SCI, SCI_BaudRate_115_2_kBaud);
    //SCI_enableAutoBaudAlign(SCI);

    SCI_enable(SCI);

    SCI_enableFifoEnh(SCI);
    SCI_resetChannels(SCI);

    //SCI_resetTxFifo(SCI);
    //SCI_clearTxFifoInt(SCI);
    //SCI_setTxFifoIntLevel(SCI, SCI_FifoLevel_1_Word);

    SCI_resetRxFifo(SCI);
    SCI_setRxFifoIntLevel(SCI, SCI_FifoLevel_2_Words);
    SCI_enableRxFifoInt(SCI);
    SCI_clearRxFifoInt(SCI);  //for a reason, Rx interupt must be cleared after the enable, probably ti's api bug.
    SCI_clearRxFifoOvf(SCI);

    //SCI_setPriority(SCI, SCI_Priority_FreeRun);


    USB_gpioInit(HandleRobot.HandleGPIO);

}

void USB_gpioInit(GPIO_Handle GPIO)
{

    GPIO_setPullUp(GPIO, GPIO_Number_28, GPIO_PullUp_Enable);
    GPIO_setPullUp(GPIO, GPIO_Number_29, GPIO_PullUp_Disable);
    GPIO_setQualification(GPIO, GPIO_Number_28, GPIO_Qual_ASync);
    GPIO_setMode(GPIO, GPIO_Number_28, GPIO_28_Mode_SCIRXDA);
    GPIO_setMode(GPIO, GPIO_Number_29, GPIO_29_Mode_SCITXDA);
}





