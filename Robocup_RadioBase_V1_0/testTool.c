/*
 * testTool.c
 *
 *  Created on: 2014-05-27
 *      Author: Mathieu Garon
 */

#include "testTool.h"

void test_is_SpiDevices_Working_Well(){
	//System_printf("%d\r\n",arduino_Test(),ARDUINO_RECALL_MESSAGE);
	bool tested = true;
	System_printf("*******SPI Device testing********\r\n");

	if(nRF_test())
		System_printf("RF [OK]\r\n");
	else{
		System_printf("RF [FAIL]\r\n");
		tested = false;
	}

	System_flush();

	if(tested)
		System_printf("SUCCESS\r\n");
	else
		System_printf("FAILURE\r\n");

	System_flush();
}


void test_Send_Command_To_Player(UnPacker_Handle *unpacker, uint16_t playerId, uint16_t commandId){

	if(commandId == 0){
		Unpacker_cleanPlayerBuffer(unpacker, playerId);
	}
	else if(commandId == 1){
		//TODO set nrf channel
		unpacker->playerReady[playerId] = true;

		unpacker->playerBuffer[playerId][0] = commandId;

		unpacker->playerBuffer[playerId][1] = 0x3f;  //float = 1
		unpacker->playerBuffer[playerId][2] = 0x80;
		unpacker->playerBuffer[playerId][3] = 0;
		unpacker->playerBuffer[playerId][4] = 0;

		unpacker->playerBuffer[playerId][5] = 0x3f;  //float = 0.5
		unpacker->playerBuffer[playerId][6] = 0;
		unpacker->playerBuffer[playerId][7] = 0;
		unpacker->playerBuffer[playerId][8] = 0;

		unpacker->playerBuffer[playerId][9] = 0x3d;    //float = 0.1
		unpacker->playerBuffer[playerId][10] = 0xcc;
		unpacker->playerBuffer[playerId][11] = 0xcc;
		unpacker->playerBuffer[playerId][12] = 0xcd;
	}

}

void test_wait(){
	uint32_t i,j = 0;
	for(i = 0; i < 10000;i++){
		for(j = 0; j < 10;j++);
	}
}
