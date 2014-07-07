/*
 * unPacker.h
 *
 *  Created on: 2014-05-23
 *      Author: Mathieu Garon
 */

#ifndef UNPACKER_H_
#define UNPACKER_H_

#include "Robocup_Define_RadioBase.h"
#include "circularBuffer.h"

UnPacker_Handle UnPacker_init();

void Unpacker_cleanPlayerBuffer(UnPacker_Handle *unpacker, uint16_t playerId);
void Unpacker_parseCircularBuffer(UnPacker_Handle *unpacker, CB_Handle *CB);
bool Unpacker_parseBuffer(UnPacker_Handle *unpacker, CB_Handle *CB);
void UnPacker_addByte(UnPacker_Handle *unpacker,int16_t byte);
void UnPacker_setupParsing(UnPacker_Handle *unpacker,uint16_t byte);
void UnPacker_printPacket(UnPacker_Handle *unpacker);

#endif /* UNPACKER_H_ */
