/*
 * two_wheel_ctrl.h
 *
 *  Created on: 2014-01-06
 *      Author: Philippe Babin
 */

#ifndef FOUR_WHEEL_CTRL_H_
#define FOUR_WHEEL_CTRL_H_

#include <stdio.h>
#include <IQmathLib.h>
#include "Robocup_Define.h"
#include "DCMotor_driver.h"

void fourWheelCtrl_Init();
void fourWheelCtrl_Update(float pX, float pY, float pTheta);

#endif /* FOUR_WHEEL_CTRL_H_ */
