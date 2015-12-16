/*
 * arduino_driver.c
 *
 *  Created on: 2013-11-08
 *      Author: Philippe Babin
 */


#include "CS_demux.h"
#include "arduino_driver.h"
#include "SPI.h"

void arduino_Init( chip_select CSPin){
	HandleArduino.CSPin = CSPin;
}

uint16_t arduino_ReadRegister( uint16_t Reg){
	uint16_t answer = 0;
	demux_connect_to(HandleArduino.CSPin);

	SPI_write_8bits(HandleRobot.HandleSPI, Reg); // Set to read
	SPI_write(HandleRobot.HandleSPI, 0x000); //dummy data
	while(SPI_getRxFifoStatus(HandleRobot.HandleSPI) < SPI_FifoStatus_2_Words); //wait for two words (STATUS + REG)
	SPI_read(HandleRobot.HandleSPI);
	answer = SPI_read(HandleRobot.HandleSPI);

	demux_disconnect();
	return answer; // Set 0 the first 2 bytes
}

void arduino_WriteRegister(uint16_t Reg, uint16_t Value){
	demux_connect_to(HandleArduino.CSPin);

	SPI_write_8bits(HandleRobot.HandleSPI, Reg | ARDUINO_WRITE_FLAG); //put a 1 on the first
	SPI_write_8bits(HandleRobot.HandleSPI, Value);
	while(SPI_getRxFifoStatus(HandleRobot.HandleSPI) < SPI_FifoStatus_2_Words); //wait for two words (STATUS + REG)
	SPI_read(HandleRobot.HandleSPI);
	SPI_read(HandleRobot.HandleSPI);

	demux_disconnect();
}


int arduino_Test(){
    uint8_t res = arduino_ReadRegister(ARDUINO_RECALL);
	//uint8_t res = arduino_ReadRegister(ARDUINO_BAT_MONITOR_1);
	System_printf("ar : %d\r\n", res);
	return res == ARDUINO_RECALL_MESSAGE;
}

