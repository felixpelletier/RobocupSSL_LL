/*
 * testTool.h
 *
 *  Created on: 2014-05-27
 *      Author: Mathieu Garon
 */

#ifndef TESTTOOL_H_
#define TESTTOOL_H_

#include "nRF24L01_driver.h"
#include "unpacker.h"

void test_is_SpiDevices_Working_Well();

void test_Send_Command_To_Player(UnPacker_Handle *unpacker, uint16_t playerId, uint16_t commandId);

void test_wait();

#endif /* TESTTOOL_H_ */
