/*
 * Robocup_Define.h
 *
 *  Created on: 2013-10-16
 *      Author: Mathieu Garon
 */

#ifndef ROBOCUP_DEFINE_H_
#define ROBOCUP_DEFINE_H_

#include <stdio.h>
#include <file.h>

#include <stdbool.h>
typedef unsigned char uint8_t;
typedef _Bool bool_t;

/*------------------------------------------------------------------------------------------------
 * 	 SYS/BIOS's files
 *------------------------------------------------------------------------------------------------*/
#include <stdint.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Swi.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>

/*------------------------------------------------------------------------------------------------
 * 	 Picollo F28027's driver files
 *------------------------------------------------------------------------------------------------*/
#include "f2802x_common/include/gpio.h"
#include "f2802x_common/include/adc.h"
#include "f2802x_common/include/clk.h"
#include "f2802x_common/include/flash.h"
#include "f2802x_common/include/pie.h"
#include "f2802x_common/include/pll.h"
#include "f2802x_common/include/wdog.h"
#include "f2802x_common/include/pwm.h"
#include "f2802x_common/include/spi.h"
#include "f2802x_common/include/cpu.h"
#include "sci.h"
#include "f2802x_common/include/sci_io.h"


/*------------------------------------------------------------------------------------------------
 * 	 Robots constant
 *------------------------------------------------------------------------------------------------*/
#define WHEEL_RADIUS 0.775
#define RRTIME 0.01		//RoundRobin call time in S
#define ENCODER_PPR 1022			//encoder pulses

/*-------------------------------------------------------------------------------------------------
 * Global constant
 --------------------------------------------------------------------------------------------------*/
#define NUMBER_OF_PLAYER 6
#define PLAYER_BUFFER_SIZE 15
#define CIRCULARBUFFER_SIZE 1024

/*------------------------------------------------------------------------------------------------
 * 	 IQ math
 *------------------------------------------------------------------------------------------------*/
/*************************
 * 	Q	       precision
 * 	23 => -256 to 255.999 999 981
 *
 *************************/
#define GLOBAL_Q 23  	//global precision
#include <IQmathLib.h>  //Virtual FPU
Void math_init();
/*------------------------------------------------------------------------------------------------
 * 	 Delay funciton (for loop 10ms for 10MHz)
 *------------------------------------------------------------------------------------------------*/
Void System_10msDelay(uint32_t ms);
Void System_100usDelay(uint32_t us);


// Work around for code that might accidently use uint8_t
//typedef unsigned char uint8_t;

typedef short             int16;
//typedef unsigned short    Uint16;

/****************************************************************************
 * 								RF MODULE
 ****************************************************************************/
typedef struct nRF24L01_Handle {
	GPIO_Number_e structCePin;
	GPIO_Number_e structCSNPin;
	uint16_t Reg[10];  //generic registers
	uint16_t TXPayload[PLAYER_BUFFER_SIZE];
	uint16_t RXPayload[PLAYER_BUFFER_SIZE];
	uint16_t TXADDR[5];
	uint16_t RXADDR[5];
	uint8_t RXCount;
	uint8_t RobotID;
}nRF24L01_Handle;

/****************************************************************************
 * 								Circular Buffer
 ****************************************************************************/
typedef struct CB_Handle{

	uint8_t byteCount;
	bool_t dataReady;
	uint16_t dataBuffer[CIRCULARBUFFER_SIZE];
	uint16_t dataBufferCursorRead;
	uint16_t dataBufferCursorWrite;

}CB_Handle;

/****************************************************************************
 * 								unpacker
 * 	Each Players have a buffer. packetSize is the actual number of data in that
 * 	buffer
 ****************************************************************************/
#define STARTBYTE 0x7E
#define ESCAPEBYTE 0x7D
#define STOPBYTE 0x7F
#define NO_PLAYER_SET -1
typedef struct UnPacker_Handle{

	uint16_t playerBuffer[NUMBER_OF_PLAYER][PLAYER_BUFFER_SIZE];
	bool playerReady[NUMBER_OF_PLAYER];

	uint16_t byteCount;
	int16_t currentPlayertoTransfer;
	bool isParsing;
	bool escapeNextByte;

}UnPacker_Handle;

/****************************************************************************
 * 								ROBOT
 ****************************************************************************/

typedef struct Robot_Handle {
	CLK_Handle HandleCLK;
	GPIO_Handle HandleGPIO;
	PIE_Handle HandlePIE;
	SPI_Handle HandleSPI;
	FLASH_Handle HandleFlash;
    CPU_Handle HandleCpu;
    PLL_Handle HandlePll;
    WDOG_Handle HandleWDog;
    nRF24L01_Handle HandleRF;
    CB_Handle HandleCB;
    UnPacker_Handle HandleUnPacker;
    SCI_Handle HandleSCI;
} Robot_Handle;

extern Robot_Handle HandleRobot;

#define HandleRF  HandleRobot.HandleRF
#define HandleSerial  HandleRobot.HandleSerial


#endif /* ROBOCUP_DEFINE_H_ */
