/*
 * nRF24L01_driver.c
 *
 *  Created on: 2013-10-24
 *      Author: Mathieu Garon
 */


/*************************************************************************************************
* \file nRF24L01.c
* \author Simon East-Lavoie
* \brief Fichier des differente commandes pour l'utilisation et la configuration du module de
*        communication sans-fil nRF24L01+
* Compilateur: Keil ARM, avec librairie StellarisWare de TI
* Carte de d�veloppement: EKS-LM3S811
* \version 1.0 2013-01-24 Premiere version pour Robocup -M.S.
**************************************************************************************************/


#include "nRF24L01_driver.h"

/**************************************************************************************************
* \fn nRFInit()
* \brief Initialisation du module nRF24L01
*
**************************************************************************************************/
void nRFInit(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI,GPIO_Number_e CePin,GPIO_Number_e CSNPin)
{
	uint8_t i = 0;
	HandleRF.structCePin = CePin;
	HandleRF.structCSNPin = CSNPin;
    // Configure gpio for RF module
    GPIO_setMode(HandleGPIO, CSNPin, GPIO_0_Mode_GeneralPurpose);  //CSN used to activate SPI communication
    GPIO_setMode(HandleGPIO, CePin, GPIO_0_Mode_GeneralPurpose);  //CE used to activate PTX or RTX mode
    GPIO_setDirection(HandleGPIO, CePin, GPIO_Direction_Output);
    GPIO_setDirection(HandleGPIO, CSNPin, GPIO_Direction_Output);
    // set pin high for normal operation
    nRF_setCSN(true,HandleGPIO);
    // set mcu memory to zero
    for(i = 0; i < 10;i++){
    	HandleRF.Reg[i] = 0;
    }
    for(i = 0; i < 10;i++){
    	HandleRF.RXADDR[i] = 0;
    	HandleRF.TXADDR[i] = 0;
    }


	nRF_setCE(false,HandleGPIO); 											 // CE is low for spi communication
	HandleRF.Reg[R_CONFIG] = TX_MODE|EN_CRC;								 //No interupts + TX mode
	nRF_WriteRegister(HandleSPI,HandleGPIO,CONFIG,HandleRF.Reg[R_CONFIG]); 	 //update nrf
	nRF_WriteRegister(HandleSPI,HandleGPIO,RF_SETUP,RATE_1MBPS|OUPUTPOWER_M6DB);
	nRF_WriteRegister(HandleSPI,HandleGPIO,RX_PW_P0,PLAYER_BUFFER_SIZE);
	nRF_WriteRegister(HandleSPI,HandleGPIO,RX_PW_P1,PLAYER_BUFFER_SIZE);
	//nRF_WriteRegister(HandleSPI,HandleGPIO,RF_CH,0x02);   				 //par defaut = 2
	nRF_WriteRegister(HandleSPI,HandleGPIO,EN_RXADDR,DATAPIPE_0); 				 //enable data pipe 0

	nRF_WriteRXADDR(HandleSPI,HandleGPIO, 0x07, RX_ADDR_P0); 				 // set the Rx adresse E0
	nRF_WriteTXADDR(HandleSPI, HandleGPIO,  0x07); 							 //set the Tx adress for ACK E7

	// Read all register for mcu use
	nRF_ReadRXADDR( HandleGPIO, HandleSPI, RX_ADDR_P0);
	nRF_ReadTXADDR( HandleGPIO, HandleSPI);
	HandleRF.Reg[R_STATUS] = nRF_ReadRegister(HandleSPI,HandleGPIO,STATUS);
	HandleRF.Reg[R_CONFIG] = nRF_ReadRegister(HandleSPI,HandleGPIO,CONFIG);
	HandleRF.Reg[R_RFSETUP] = nRF_ReadRegister(HandleSPI,HandleGPIO,RF_SETUP);
	HandleRF.Reg[R_PLLength0] = nRF_ReadRegister(HandleSPI,HandleGPIO,RX_PW_P0);
	HandleRF.Reg[R_PLLength1] = nRF_ReadRegister(HandleSPI,HandleGPIO,RX_PW_P1);
	HandleRF.Reg[R_RFChannel] = nRF_ReadRegister(HandleSPI,HandleGPIO,RF_CH);
	HandleRF.Reg[R_ENRXADDR] = nRF_ReadRegister(HandleSPI,HandleGPIO,EN_RXADDR);
	HandleRF.Reg[R_FIFOSTATUS] = nRF_ReadRegister(HandleSPI,HandleGPIO,FIFO_STATUS);

	nRF_PowerUp(HandleSPI,HandleGPIO,true);
	System_10msDelay(1);
}




/**************************************************************************************************
* Set enable pin (PTX/PRX control)
**************************************************************************************************/
void nRF_setCE(bool bEnable,GPIO_Handle HandleGPIO)
{

	if(bEnable)
		GPIO_setHigh(HandleGPIO, HandleRF.structCePin);
	else
		GPIO_setLow(HandleGPIO, HandleRF.structCePin);
}
/**************************************************************************************************
* Set Csn pin (spi control)
**************************************************************************************************/
void nRF_setCSN(bool bEnable,GPIO_Handle HandleGPIO)
{
	if(bEnable)
		GPIO_setHigh(HandleGPIO, HandleRF.structCSNPin);
	else
		GPIO_setLow(HandleGPIO, HandleRF.structCSNPin);
}

uint16_t nRF_ReadRegister(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO,uint16_t Reg){
	uint16_t answer = 0;
	nRF_setCSN(false,HandleGPIO); //Set csn for spi communication
	SPI_write(HandleSPI, Reg); //register adress
	SPI_write(HandleSPI, Reg); //dummy data
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_2_Words); //wait for two words (STATUS + REG)
	HandleRF.Reg[R_STATUS] = SPI_read(HandleSPI);
	answer = SPI_read(HandleSPI);
	nRF_setCSN(true,HandleGPIO);
	return answer;
}

void nRF_WriteRegister (SPI_Handle HandleSPI,GPIO_Handle HandleGPIO,uint16_t Reg,uint16_t value)
{
	nRF_setCSN(false,HandleGPIO);
	SPI_write(HandleSPI,W_REGISTER | Reg); //register adress
	SPI_write(HandleSPI, value << 8);
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_2_Words); //wait for status and junk
	HandleRF.Reg[R_STATUS] = SPI_read(HandleSPI);
	HandleRF.Reg[R_JUNK] = SPI_read(HandleSPI);
	nRF_setCSN(true,HandleGPIO);
}

void nRF_sendPackets(UnPacker_Handle *buffers){
	uint16_t i = 0;
	for(i=0; i < NUMBER_OF_PLAYER; ++i){

		uint16_t j = 0;
		if(buffers->playerReady[i]){
			//System_printf("\n\rSend:");
			for(j = 0; j < PLAYER_BUFFER_SIZE; ++j){
				HandleRF.TXPayload[j] = buffers->playerBuffer[i][j];
				System_printf("%d",HandleRF.TXPayload[j]);
			}
			System_printf("\n");
			nRF_WriteTXPayload(HandleRobot.HandleGPIO,HandleRobot.HandleSPI,PLAYER_BUFFER_SIZE);
			nRF_setCE(true,HandleRobot.HandleGPIO);
			while(!nRF_payloadSent() && !nRF_maxRetransmit());

			nRF_setCE(false,HandleRobot.HandleGPIO);
			Unpacker_cleanPlayerBuffer(buffers, i);
		}
		nRF_resetTransmitStatus();
	}
	nRF_setCE(false,HandleRobot.HandleGPIO);
}

// check if a payload was sent
bool nRF_payloadSent(){
	HandleRF.Reg[R_STATUS] = nRF_ReadRegister(HandleRobot.HandleSPI,HandleRobot.HandleGPIO,STATUS);
	if(HandleRF.Reg[R_STATUS] & 0x20){
		return true;
	}
	else{
		return false;
	}
}

bool nRF_maxRetransmit(){
	HandleRF.Reg[R_STATUS] = nRF_ReadRegister(HandleRobot.HandleSPI,HandleRobot.HandleGPIO,STATUS);
	if(HandleRF.Reg[R_STATUS] & 0x10){
		return true;
	}
	else{
		return false;
	}
}


void nRF_resetTransmitStatus(){
	if(HandleRF.Reg[R_STATUS] & 0x10){  //maximum of Tx retransmit bit
		nRF_WriteRegister(HandleRobot.HandleSPI, HandleRobot.HandleGPIO, STATUS, 0x10);
	}
	if(HandleRF.Reg[R_STATUS] & 0x20){ // data sent tx fifo
		nRF_WriteRegister(HandleRobot.HandleSPI, HandleRobot.HandleGPIO, STATUS, 0x20);
	}
	int i = 0;
	for(i = 0; i < PLAYER_BUFFER_SIZE; ++i){
		HandleRF.TXPayload[i] = 0;
	}
}

bool nRF_isTxFull(){
	HandleRF.Reg[R_FIFOSTATUS] = nRF_ReadRegister(HandleRobot.HandleSPI, HandleRobot.HandleGPIO, FIFO_STATUS);
	if(HandleRF.Reg[R_FIFOSTATUS] & 0x0200){ //if Tx is full
		return true;
	}
	else{
		return false;
	}
}

/**************************************************************************************************
* \fn nRF_WriteTXPayload()
* \brief Send wireless packet
* Send block of 4 to be sure de SPI fifo is not overflown
*
**************************************************************************************************/
void nRF_WriteTXPayload(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI,uint16_t payloadLenght)
{
	uint8_t i=0;
	uint8_t j=0;
	uint16_t entireBlock = payloadLenght/4;
	uint16_t restBlock = payloadLenght%4;

	nRF_setCSN(false,HandleGPIO); 	// CSN pin LOW

	SPI_write(HandleSPI, W_TX_PAYLOAD);
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_1_Word);
	HandleRF.Reg[R_STATUS] = SPI_read(HandleSPI);
	for(i = 0; i < entireBlock; ++i){
		for(j=0; j<4; ++j){SPI_write(HandleSPI, HandleRF.TXPayload[j+(4*i)]<<8);}
		for(j=0;j<4;++j){
			while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_1_Word);
			HandleRF.Reg[R_JUNK] = SPI_read(HandleSPI);
		}
	}
	for(j=0; j<restBlock; ++j){SPI_write(HandleSPI, HandleRF.TXPayload[j+(4*entireBlock)]<<8);}
	for(i = 0; i < restBlock; ++i){
		while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_1_Word);
		HandleRF.Reg[R_JUNK] = SPI_read(HandleSPI);
	}

	nRF_setCSN(true,HandleGPIO);
	//System_100usDelay(100);
	//nRF_setCE(true,HandleGPIO);  //minimum 10 us pulse to send data
	//System_100usDelay(100);
	//nRF_setCE(false,HandleGPIO);
}





/**************************************************************************************************
* \fn nRF_ReadRXPayload()
* \brief Receive wireless paket from nRF24L01
*
**************************************************************************************************/
void nRF_ReadRXPayload(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI)
{
		uint8_t i=0;
		nRF_setCSN(false,HandleGPIO); 																																																// CE pin LOW

		for(i=0; i<6; i++){SPI_write(HandleSPI, R_RX_PAYLOAD);}
		while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_1_Word); //wait??// Command + dummy data
		HandleRF.Reg[R_STATUS] = SPI_read(HandleSPI);																											// Read received payload
		while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_2_Words); //wait
		HandleRF.RXPayload[0] = SPI_read(HandleSPI);
		HandleRF.RXPayload[1] = SPI_read(HandleSPI);
		while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_3_Words); //wait
		HandleRF.RXPayload[2] = SPI_read(HandleSPI);
		HandleRF.RXPayload[3] = SPI_read(HandleSPI);
		HandleRF.RXPayload[4] = SPI_read(HandleSPI);

		nRF_setCSN(true,HandleGPIO);
																																																		// Count the received packets
}

/**************************************************************************************************
* \fn nRF_ReadRXPayload()
* \brief Receive wireless paket from nRF24L01
*
**************************************************************************************************/
void nRF_ReadTXPayload(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI)
{
		uint8_t i=0;
		nRF_setCSN(false,HandleGPIO); 																																																// CE pin LOW

		for(i=0; i<6; i++){SPI_write(HandleSPI, R_RX_PAYLOAD);}
		while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_1_Word); //wait??// Command + dummy data
		HandleRF.Reg[R_STATUS] = SPI_read(HandleSPI);																											// Read received payload
		while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_2_Words); //wait
		HandleRF.RXPayload[0] = SPI_read(HandleSPI);
		HandleRF.RXPayload[1] = SPI_read(HandleSPI);
		while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_3_Words); //wait
		HandleRF.RXPayload[2] = SPI_read(HandleSPI);
		HandleRF.RXPayload[3] = SPI_read(HandleSPI);

		nRF_setCSN(true,HandleGPIO);
																																																		// Count the received packets
}

/**************************************************************************************************
*  Reading paket procedur
**************************************************************************************************/

Bool nRF_Listen(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI){
	uint16_t interupt = 0;
	uint16_t fifoEmpty = 0;

	nRF_setCE(false, HandleGPIO);

	HandleRF.Reg[R_FIFOSTATUS] = nRF_ReadRegister(HandleSPI,HandleGPIO,FIFO_STATUS); //update FIFO status
	fifoEmpty = HandleRF.Reg[R_FIFOSTATUS] & 0x01; //bit mask if fifo is empty
	interupt = HandleRF.Reg[R_STATUS] & 0x40; //bit mask to watch if receive interupt is up
	if(interupt == 0x00){
		nRF_setCE(true, HandleGPIO);
		return false;
	}
	else{
		while(!fifoEmpty){ //cette boucle garde seulement la derniere transmission
			nRF_ReadRXPayload(HandleGPIO,HandleSPI);
			HandleRF.Reg[R_FIFOSTATUS] = nRF_ReadRegister(HandleSPI,HandleGPIO,FIFO_STATUS); //update FIFO status
			fifoEmpty = HandleRF.Reg[R_FIFOSTATUS] & 0x01; //bit mask if fifo is empty
		}
		nRF_WriteRegister(HandleSPI,HandleGPIO,STATUS, 0x40); //set the flag down
		nRF_setCE(true, HandleGPIO);
		return true;
	}
}


/**************************************************************************************************
* \fn nRF_ReadTXADDR()
* \brief Reads the nRF24L01's TX_ADDR register
*
**************************************************************************************************/
void nRF_ReadTXADDR(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI)
{
	uint8_t i=0;

	nRF_setCSN(false,HandleGPIO);

	for(i=0; i<6; i++){SPI_write(HandleSPI, TX_ADDR); }
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_1_Word); //wait??
	HandleRF.Reg[R_STATUS] = SPI_read(HandleSPI);
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_2_Words); //wait??
	HandleRF.TXADDR[0] = SPI_read(HandleSPI);
	HandleRF.TXADDR[1] = SPI_read(HandleSPI);
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_3_Words); //wait??
	HandleRF.TXADDR[2] = SPI_read(HandleSPI);
	HandleRF.TXADDR[3] = SPI_read(HandleSPI);
	HandleRF.TXADDR[4] = SPI_read(HandleSPI);

	nRF_setCSN(true,HandleGPIO);
}

void nRF_ReadRXADDR(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI, uint16_t pipe)
{
	uint8_t i=0;

	nRF_setCSN(false,HandleGPIO);

	for(i=0; i<6; i++){SPI_write(HandleSPI, pipe); }
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_1_Word); //wait??
	HandleRF.Reg[R_STATUS] = SPI_read(HandleSPI);
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_2_Words); //wait??
	HandleRF.RXADDR[0] = SPI_read(HandleSPI);
	HandleRF.RXADDR[1] = SPI_read(HandleSPI);
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_3_Words); //wait??
	HandleRF.RXADDR[2] = SPI_read(HandleSPI);
	HandleRF.RXADDR[3] = SPI_read(HandleSPI);
	HandleRF.RXADDR[4] = SPI_read(HandleSPI);

	nRF_setCSN(true,HandleGPIO);
}





/**************************************************************************************************
* \fn nRF_WriteTXADDR(unsigned int uiID)
* \brief Write TXADDR reg en fonction de l'ID du robot voul
*
**************************************************************************************************/
void nRF_WriteTXADDR(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO, uint8_t uiID)
{
	unsigned int i=0;


	nRF_setCSN(false,HandleGPIO);

	SPI_write(HandleSPI, W_REGISTER | TX_ADDR); //register adress
	for(i=0; i<4; i++){SPI_write(HandleSPI, 0xE700);}SPI_write(HandleSPI, 0xE000 | (uiID<<8));
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_1_Word); //wait
	HandleRF.Reg[R_STATUS] = SPI_read(HandleSPI);
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_2_Words); //wait
	HandleRF.Reg[R_JUNK]= SPI_read(HandleSPI);
	HandleRF.Reg[R_JUNK] = SPI_read(HandleSPI);
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_3_Words); //wait
	HandleRF.Reg[R_JUNK] = SPI_read(HandleSPI);
	HandleRF.Reg[R_JUNK] = SPI_read(HandleSPI);
	HandleRF.Reg[R_JUNK] = SPI_read(HandleSPI);

	nRF_setCSN(true,HandleGPIO);
}

void nRF_WriteRXADDR(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO, uint8_t uiID, uint16_t pipe)
{
	unsigned int i=0;


	nRF_setCSN(false,HandleGPIO);

	SPI_write(HandleSPI, W_REGISTER | pipe); //register adress
	for(i=0; i<4; i++){SPI_write(HandleSPI, 0xE700);}SPI_write(HandleSPI, 0xE000 | (uiID<<8));
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_1_Word); //wait
	HandleRF.Reg[R_STATUS] = SPI_read(HandleSPI);
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_2_Words); //wait
	HandleRF.Reg[R_JUNK]= SPI_read(HandleSPI);
	HandleRF.Reg[R_JUNK] = SPI_read(HandleSPI);
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_3_Words); //wait
	HandleRF.Reg[R_JUNK] = SPI_read(HandleSPI);
	HandleRF.Reg[R_JUNK] = SPI_read(HandleSPI);
	HandleRF.Reg[R_JUNK] = SPI_read(HandleSPI);

	nRF_setCSN(true,HandleGPIO);
}

void nRF_PowerUp(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO, Bool ans){



	if(ans)
		HandleRF.Reg[R_CONFIG] = HandleRF.Reg[R_CONFIG] | 0x02;
	else
		HandleRF.Reg[R_CONFIG] = HandleRF.Reg[R_CONFIG] & ~0x02;

	nRF_WriteRegister(HandleSPI,HandleGPIO,CONFIG,HandleRF.Reg[R_CONFIG]); //Power up
}










/**************************************************************************************************
* \fn nRF_FlushRX()
* \brief Flush RX FIFO
*
**************************************************************************************************/
void nRF_FlushRX(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO)
{
	nRF_setCSN(false,HandleGPIO);
	SPI_write(HandleSPI, FLUSH_RX);
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_1_Word); //wait
	HandleRF.Reg[R_STATUS] = SPI_read(HandleSPI);
	nRF_setCSN(true,HandleGPIO);
}



/**************************************************************************************************
* \fn nRF_FlushTX()
* \brief Flush TX FIFO
*
**************************************************************************************************/
void nRF_FlushTX(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO)
{
	nRF_setCSN(false,HandleGPIO);
	SPI_write(HandleSPI, FLUSH_TX);
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_1_Word); //wait
	HandleRF.Reg[R_STATUS] = SPI_read(HandleSPI);
	nRF_setCSN(true,HandleGPIO);
}


void nRF_DisplayRegister(void)
{
	System_printf("Status = 0x%x\n\r",HandleRF.Reg[R_STATUS]);
	System_printf("Config = 0x%x\n\r",HandleRF.Reg[R_CONFIG]);
	System_printf("RF_Setup = 0x%x\n\r",HandleRF.Reg[R_RFSETUP]);
	System_printf("PLLength0 = 0x%x\n\r",HandleRF.Reg[R_PLLength0]);
	System_printf("PLLength1 = 0x%x\n\r",HandleRF.Reg[R_PLLength1]);
	System_printf("RF_CH = 0x%x\n\r",HandleRF.Reg[R_RFChannel]);
	System_printf("EN_RXADDR = 0x%x\n\r",HandleRF.Reg[R_ENRXADDR]);
	System_printf("FIFO_Status = 0x%x\n\r",HandleRF.Reg[R_FIFOSTATUS]);
}

void nRF_DisplayRXAdress(void){
	System_printf("RX add = 0x%x 0x%x 0x%x 0x%x 0x%x \n\r",HandleRF.RXADDR[0],
			HandleRF.RXADDR[1],HandleRF.RXADDR[2],HandleRF.RXADDR[3],HandleRF.RXADDR[4]);
}

void nRF_DisplayTXAdress(void){
	System_printf("TX add = 0x%x 0x%x 0x%x 0x%x 0x%x \n\r",HandleRF.TXADDR[0],
			HandleRF.TXADDR[1],HandleRF.TXADDR[2],HandleRF.TXADDR[3],HandleRF.TXADDR[4]);
}

void nRF_DisplayRXPayload(void){
	System_printf("Payload = 0x%x 0x%x 0x%x 0x%x 0x%x \n\r",HandleRF.RXPayload[0],
			HandleRF.RXPayload[1],HandleRF.RXPayload[2],HandleRF.RXPayload[3],HandleRF.RXPayload[4]);
}
void nRF_DisplayTXPayload(void){
	System_printf("Payload = 0x%x 0x%x 0x%x 0x%x 0x%x \n\r",HandleRF.TXPayload[0],
			HandleRF.TXPayload[1],HandleRF.TXPayload[2],HandleRF.TXPayload[3],HandleRF.TXPayload[4]);
}

void nRF_DisplayRetransmit(){
	System_printf("retransmit = %d\n\r",nRF_ReadRegister(HandleRobot.HandleSPI,
								HandleRobot.HandleGPIO, OBSERVE_TX));
}

bool nRF_test(){
	return nRF_ReadRegister(HandleRobot.HandleSPI,HandleRobot.HandleGPIO,CONFIG) == HandleRF.Reg[R_CONFIG]; //after init
}


