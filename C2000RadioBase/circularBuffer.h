/*
 * circularBuffer.h
 *
 *  Created on: 2014-05-23
 *      Author: Dominic Bilodeau
 */

#ifndef CIRCULARBUFFER_H_
#define CIRCULARBUFFER_H_

#include "Robocup_Define_RadioBase.h"

CB_Handle CB_init();

inline bool CB_write(CB_Handle *circularBuffer,uint16_t pByte){
	uint16_t cursorNextPos = (circularBuffer->dataBufferCursorWrite + 1) % CIRCULARBUFFER_SIZE;
	if(cursorNextPos == circularBuffer->dataBufferCursorRead)
	{
		System_printf("\n\rOverflow");
		return false;
	}
	else{
		circularBuffer->dataBuffer[circularBuffer->dataBufferCursorWrite] = pByte;
		circularBuffer->dataBufferCursorWrite = cursorNextPos;
		//System_printf("\n\r%d",circularBuffer->dataBufferCursorWrite);
		//System_printf("%x", pByte);
		return true;
	}
}

bool CB_read(CB_Handle *circularBuffer, uint16_t *pByte);
void CB_flush(CB_Handle *circularBuffer);
void CB_print(CB_Handle *circularBuffer);

#endif /* CIRCULARBUFFER_H_ */
