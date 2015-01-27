/*
 * radio_unpacker.h
 *
 *  Created on: 2014-06-03
 *      Author: Mathieu Garon
 */

#ifndef RADIO_UNPACKER_H_
#define RADIO_UNPACKER_H_

#include "Robocup_Define.h"

void unpackBuffer(uint8_t buffer[PLAYER_BUFFER_SIZE]);
void setVelocityCommand(uint8_t buffer[12]);

#endif /* RADIO_UNPACKER_H_ */
