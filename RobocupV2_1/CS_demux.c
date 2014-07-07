/*
 * CS_demux.c
 *
 *  Created on: 2014-01-09
 *      Author: Philippe Babin
 */


#include "CS_demux.h"


void demux_Init( GPIO_Number_e a0, GPIO_Number_e a1, GPIO_Number_e a2, chip_select notConnect){

	HandleDemux.a0 = a0;      // mux select pins
	HandleDemux.a1 = a1;
	HandleDemux.a2 = a2;
	HandleDemux.notConnect_pin = notConnect;

    GPIO_setMode(HandleRobot.HandleGPIO, a0, GPIO_0_Mode_GeneralPurpose); //pin activation
    GPIO_setDirection(HandleRobot.HandleGPIO, a0, GPIO_Direction_Output);
    GPIO_setMode(HandleRobot.HandleGPIO, a1, GPIO_0_Mode_GeneralPurpose);
    GPIO_setDirection(HandleRobot.HandleGPIO, a1, GPIO_Direction_Output);
    GPIO_setMode(HandleRobot.HandleGPIO, a2, GPIO_0_Mode_GeneralPurpose);
    GPIO_setDirection(HandleRobot.HandleGPIO, a2, GPIO_Direction_Output);

    demux_disconnect();
}

// this function activate the right combination of select pin for output from CS_0 to CS_7
void demux_connect_to( chip_select CS){
	if(CS == CS_0){
		GPIO_setLow(HandleRobot.HandleGPIO, HandleDemux.a0);
		GPIO_setLow(HandleRobot.HandleGPIO, HandleDemux.a1);
		GPIO_setLow(HandleRobot.HandleGPIO, HandleDemux.a2);
	}
	else if(CS == CS_1){
		GPIO_setHigh(HandleRobot.HandleGPIO, HandleDemux.a0);
		GPIO_setLow(HandleRobot.HandleGPIO, HandleDemux.a1);
		GPIO_setLow(HandleRobot.HandleGPIO, HandleDemux.a2);
	}
	else if(CS == CS_2){
		GPIO_setLow(HandleRobot.HandleGPIO, HandleDemux.a0);
		GPIO_setHigh(HandleRobot.HandleGPIO, HandleDemux.a1);
		GPIO_setLow(HandleRobot.HandleGPIO, HandleDemux.a2);
	}
	else if(CS == CS_3){
		GPIO_setHigh(HandleRobot.HandleGPIO, HandleDemux.a0);
		GPIO_setHigh(HandleRobot.HandleGPIO, HandleDemux.a1);
		GPIO_setLow(HandleRobot.HandleGPIO, HandleDemux.a2);
	}
	else if(CS == CS_4){
		GPIO_setLow(HandleRobot.HandleGPIO, HandleDemux.a0);
		GPIO_setLow(HandleRobot.HandleGPIO, HandleDemux.a1);
		GPIO_setHigh(HandleRobot.HandleGPIO, HandleDemux.a2);
	}
	else if(CS == CS_5){
		GPIO_setHigh(HandleRobot.HandleGPIO, HandleDemux.a0);
		GPIO_setLow(HandleRobot.HandleGPIO, HandleDemux.a1);
		GPIO_setHigh(HandleRobot.HandleGPIO, HandleDemux.a2);
	}
	else if(CS == CS_6){
		GPIO_setLow(HandleRobot.HandleGPIO, HandleDemux.a0);
		GPIO_setHigh(HandleRobot.HandleGPIO, HandleDemux.a1);
		GPIO_setHigh(HandleRobot.HandleGPIO, HandleDemux.a2);
	}
	else if(CS == CS_7){
		GPIO_setHigh(HandleRobot.HandleGPIO, HandleDemux.a0);
		GPIO_setHigh(HandleRobot.HandleGPIO, HandleDemux.a1);
		GPIO_setHigh(HandleRobot.HandleGPIO, HandleDemux.a2);
	}
}

// this function use the unasigned pin when there is no communication
void demux_disconnect(){
	demux_connect_to(HandleDemux.notConnect_pin);
}
