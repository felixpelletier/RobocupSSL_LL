/*
 * radio_unpacker.c
 *
 *  Created on: 2014-06-03
 *      Author: Mathieu Garon
 */

#include "radio_unpacker.h"

void unpackBuffer(uint8_t buffer[PLAYER_BUFFER_SIZE]){
	switch (buffer[0]){
	case 1:
		setVelocityCommand(&buffer[1]);
		break;
	case 2:
		break;
	case 3:
		kickCommand(&buffer[1]);
		break;
	}
}

void setVelocityCommand(uint8_t buffer[12]){

	HandleRobot.robotParam.XVelocityCommand.bytes[0] = (buffer[1]<<8) | buffer[0];  //data convert byte to float with union struct
	HandleRobot.robotParam.XVelocityCommand.bytes[1] = (buffer[3]<<8) | buffer[2];  //see "union struct" on wiki

	HandleRobot.robotParam.YVelocityCommand.bytes[0] = (buffer[5]<<8) | buffer[4];
	HandleRobot.robotParam.YVelocityCommand.bytes[1] = (buffer[7]<<8) | buffer[6];

	HandleRobot.robotParam.ThetaVelocityCommand.bytes[0] = (buffer[9]<<8) | buffer[8];
	HandleRobot.robotParam.ThetaVelocityCommand.bytes[1] = (buffer[11]<<8) | buffer[10];

	//System_printf("%x %x %x %x", buffer[9],buffer[8],buffer[11],buffer[10]);
}

void kickCommand(uint8_t buffer[12]){
	int duration = (buffer[1]<<8) | buffer[0];
	kicker_activate(duration);
	System_printf("kick %d\r\n", duration);
}

