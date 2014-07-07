/*
 * unPacket.c
 *
 *  Created on: 2014-05-23
 *      Author: Mathieu Garon
 */

#include "unpacker.h"

UnPacker_Handle UnPacker_init(){
	int playerId = 0;
	UnPacker_Handle unpackerBuffer;

	for(playerId = 0; playerId < NUMBER_OF_PLAYER; ++playerId){
		Unpacker_cleanPlayerBuffer(&unpackerBuffer,playerId);
	}
	unpackerBuffer.currentPlayertoTransfer = -1;
	unpackerBuffer.isParsing = false;
	unpackerBuffer.escapeNextByte = false;
	unpackerBuffer.byteCount = 0;

	return unpackerBuffer;
}

void Unpacker_cleanPlayerBuffer(UnPacker_Handle *unpacker, uint16_t playerId){
	unpacker->playerReady[playerId] = false;
	int i = 0;
	for(i = 0; i < PLAYER_BUFFER_SIZE; ++i){
		unpacker->playerBuffer[playerId][i] = 0;
	}
}



bool Unpacker_parseBuffer(UnPacker_Handle *unpacker, CB_Handle *CB){
	uint16_t byte = 0;
	//	bool end = false;

	while(CB_read(CB,&byte)){
		//System_printf("byte: %x\n\r",byte);
		if(unpacker->isParsing){

			if (unpacker->escapeNextByte){
				UnPacker_addByte(unpacker, byte);
				unpacker->escapeNextByte = false;
			}
			else{
				if (byte == ESCAPEBYTE){
					unpacker->escapeNextByte = true;
				}
				else if(byte == STARTBYTE){  //restart communication
					unpacker->byteCount = 0;
					unpacker->currentPlayertoTransfer = NO_PLAYER_SET;
				}
				else if(byte == STOPBYTE){
					unpacker->byteCount = 0;
					unpacker->isParsing = false;
					unpacker->playerReady[unpacker->currentPlayertoTransfer] = true;
					return true;
				}
				else{
					UnPacker_addByte(unpacker, byte);
				}
			}
		}
		else{
			UnPacker_setupParsing(unpacker, byte);
		}
	}

	return false;
}

void UnPacker_addByte(UnPacker_Handle *unpacker,int16_t byte){
	if(unpacker->currentPlayertoTransfer == NO_PLAYER_SET){
		unpacker->currentPlayertoTransfer = byte;
	}
	else{
		unpacker->playerBuffer[unpacker->currentPlayertoTransfer][unpacker->byteCount] = byte;
		unpacker->byteCount++;
	}
}

void UnPacker_setupParsing(UnPacker_Handle *unpacker,uint16_t byte){
	if(byte == STARTBYTE){
		unpacker->isParsing = true;
		unpacker->currentPlayertoTransfer = NO_PLAYER_SET;
	}
}

void UnPacker_printPacket(UnPacker_Handle *unpacker){
	int i = 0;
	int j = 0;
	for(i = 0; i < NUMBER_OF_PLAYER; ++i){
		System_printf("Player %d: ",i);
		for(j = 0; j < PLAYER_BUFFER_SIZE; ++j){
			System_printf("%x ",unpacker->playerBuffer[i][j]);
		}
		System_printf("\n\r");
		System_flush();
	}
}
