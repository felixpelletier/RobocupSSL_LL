/*
 * L3GD20_driver.h
 *
 *  Created on: 2013-11-08
 *      Author: Philippe Babin
 */

#ifndef DCMOTOR_DRIVER_H_
#define DCMOTOR_DRIVER_H_

#include <stdio.h>
#include "Robocup_Define.h"


typedef enum
{
	PWM_1A,
	PWM_1B,
	PWM_2A,
	PWM_2B,
	PWM_3A,
	PWM_3B
} DCMotor_pwm_e;

typedef enum
{
	RIGHT,
	LEFT
} DCMotor_DIR;

#define EPWM_TIMER_TBPRD  1500  // Period register
#define EPWM_BRAKE		  2000
#define EPWM_MAX_CMPA     1450
#define EPWM_MIN_CMPA       50

/*******************************************************************************
 * User Interface Function
 *******************************************************************************/

DCMotor_Handle dcMotor_init(DCMotor_pwm_e pPwmId,GPIO_Number_e pDirectionPin);

void dcMotor_updatePWM(DCMotor_Handle *pMotor, uint16_t pPidValue);
void dcMotor_setPWM(DCMotor_Handle *pMotor, uint16_t pPwm);
uint16_t dcMotor_getPWM(DCMotor_Handle* pMotor);
void dcMotor_setDirection(DCMotor_Handle *pMotor,DCMotor_DIR pDirection);
void dcMotor_break(DCMotor_Handle *pMotor);
void dcMotor_update(DCMotor_Handle* pMotor, PID_Handle *pPid);



#endif /* DCMOTOR_DRIVER_H_ */
