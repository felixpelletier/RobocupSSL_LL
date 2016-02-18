/*
 * circularBuffer.c
 *
 *  Created on: 2014-05-23
 *      Author: Dominic Bilodeau
 */

#include "circularBuffer.h"

CB_Handle CB_init(){
	CB_Handle cbHandle;
	cbHandle.byteCount = 0;
	cbHandle.dataBufferCursorRead = 0;
	cbHandle.dataBufferCursorWrite = 0;
	uint32_t i = 0;
	for(i = 0; i < CIRCULARBUFFER_SIZE; ++i){
		cbHandle.dataBuffer[i] = 0;
	}

	return cbHandle;
}



bool CB_read(CB_Handle *circularBuffer, uint16_t *pByte){
	if(circularBuffer->dataBufferCursorWrite == circularBuffer->dataBufferCursorRead){
		return false;
	}
	else{
		*pByte = circularBuffer->dataBuffer[circularBuffer->dataBufferCursorRead];
		circularBuffer->dataBufferCursorRead =
				(circularBuffer->dataBufferCursorRead + 1) % CIRCULARBUFFER_SIZE;
		return true;
	}
}

void CB_flush(CB_Handle *circularBuffer){
	circularBuffer->dataBufferCursorRead = 0;
	circularBuffer->dataBufferCursorWrite = 0;
	uint32_t i = 0;
	for(i = 0; i < CIRCULARBUFFER_SIZE; ++i){
		circularBuffer->dataBuffer[i] = 0;
	}
}

void CB_print(CB_Handle *circularBuffer){
	int i = 0;
	for(i = 0; i < CIRCULARBUFFER_SIZE; ++i){
		System_printf("%d",circularBuffer->dataBuffer[i]);
		System_printf(" ");
		System_flush();
	}

	System_printf("\n\r");
}

