/*
 * testTool.h
 *
 *  Created on: 2014-03-29
 *      Author: Mathieu Garon
 */

#ifndef TESTTOOL_H_
#define TESTTOOL_H_

#include "quad_driver.h"
#include "arduino_driver.h"
#include "nRF24L01_driver.h"
#include "L3GD20_driver.h"
#include "LSM9DS0_driver.h"
#include "nRF24L01_driver.h"
#include "DCMotor_driver.h"

void test_is_SpiDevices_Working_Well();
void test_is_Wheel_System_Working_Well();

void test_is_All_Encoder_Are_Matched_With_Wheels();
bool test_is_Encoder_is_Matched_With_Wheel(uint8_t pMotorId);

void test_is_Directions_Ok();
bool test_is_Wheel_Direction_Left(uint8_t pMotorId);
bool test_is_Wheel_Direction_Right(uint8_t pMotorId);

void test_wait();

#endif /* TESTTOOL_H_ */
