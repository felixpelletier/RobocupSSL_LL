/*
 * arduino_driver.c
 *
 *  Created on: 2013-11-08
 *      Author: Philippe Babin
 */


#include "CS_demux.h"
#include "arduino_driver.h"


void arduino_Init( chip_select CSPin){
	HandleArduino.CSPin = CSPin;
}

uint16_t arduino_ReadRegister( uint16_t Reg){
	uint16_t answer = 0;
	demux_connect_to(HandleArduino.CSPin);

	SPI_write(HandleRobot.HandleSPI, Reg); // Set to read
	SPI_write(HandleRobot.HandleSPI, 0xFFFF); //dummy data
	while(SPI_getRxFifoStatus(HandleRobot.HandleSPI) < SPI_FifoStatus_2_Words); //wait for two words (STATUS + REG)
	answer = SPI_read(HandleRobot.HandleSPI);
	SPI_read(HandleRobot.HandleSPI);

	demux_disconnect();
	return answer; // Set 0 the first 2 bytes
}

void arduino_WriteRegister(uint16_t Reg, uint16_t Value){
	demux_connect_to(HandleArduino.CSPin);

	SPI_write(HandleRobot.HandleSPI, Reg); //put a 0 on the first
	SPI_write(HandleRobot.HandleSPI, Value);
	while(SPI_getRxFifoStatus(HandleRobot.HandleSPI) < SPI_FifoStatus_2_Words); //wait for two words (STATUS + REG)
	SPI_read(HandleRobot.HandleSPI);
	SPI_read(HandleRobot.HandleSPI);

	demux_disconnect();
}


int arduino_Test(){
	return arduino_ReadRegister(ARDUINO_RECALL) << 8 == ARDUINO_RECALL_MESSAGE;
}

