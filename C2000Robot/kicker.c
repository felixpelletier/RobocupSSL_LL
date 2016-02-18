/*
 * kicker.c
 *
 *  Created on: 2016-02-04
 *      Author: robocup
 */

#include "kicker.h"


void kicker_init( GPIO_Number_e kick_pin){
	HandleKicker.kick_pin = kick_pin;
	HandleKicker.activated = false;
	HandleKicker.duration = 0;

	GPIO_setMode(HandleRobot.HandleGPIO, kick_pin, GPIO_0_Mode_GeneralPurpose); //pin activation
    GPIO_setDirection(HandleRobot.HandleGPIO, kick_pin, GPIO_Direction_Output);
	GPIO_setLow(HandleRobot.HandleGPIO, HandleKicker.kick_pin);
}

// Call when the base station want to kick
void kicker_activate(int duration){
	if(!HandleKicker.activated){
		HandleKicker.activated = true;
		HandleKicker.duration = duration;
		GPIO_setHigh(HandleRobot.HandleGPIO, HandleKicker.kick_pin);
	}

}

// Call at each round robin to update the state
void kicker_update(){
	if(HandleKicker.activated){
		System_printf("kick activated%d\r\n", HandleKicker.duration);
		HandleKicker.duration -= 2;
		if(HandleKicker.duration == 0){
			HandleKicker.activated = false;
			GPIO_setLow(HandleRobot.HandleGPIO, HandleKicker.kick_pin);
		}
	}
}
