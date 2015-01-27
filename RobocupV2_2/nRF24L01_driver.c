/*
 * nRF24L01_driver.c
 * Radio frequency communicator
 *
 *  Created on: 2013-10-24
 *      Author: Mathieu Garon
 */

#include "CS_demux.h"
#include "nRF24L01_driver.h"

/**************************************************************************************************
* \fn nRFInit()
* \brief Initialisation du module nRF24L01
*
**************************************************************************************************/
void nRFInit(GPIO_Handle HandleGPIO, SPI_Handle HandleSPI, GPIO_Number_e CePin, chip_select MuxValue)
{
	uint8_t i = 0;
	HandleRF.structCePin = CePin;
	HandleRF.structCSNPin = MuxValue;
    // Configure gpio for RF module
    GPIO_setMode(HandleGPIO, CePin, GPIO_0_Mode_GeneralPurpose);  //CE used to activate PTX or RTX mode
    GPIO_setDirection(HandleGPIO, CePin, GPIO_Direction_Output);
    // set pin high for normal operation
    demux_disconnect();

    nRF_flushBuffers();

	nRF_setCE(false,HandleGPIO); 											 // CE is low for spi communication
	nRF_WriteRegister(HandleSPI,HandleGPIO,RX_PW_P0,PLAYER_BUFFER_SIZE);  				 //payload 0 = 3 byte
	//nRF_WriteRegister(HandleSPI,HandleGPIO,RX_PW_P1,PLAYER_BUFFER_SIZE);  				 //payload 1 = 5 byte
	//nRF_WriteRegister(HandleSPI,HandleGPIO,RF_CH,0x01);   				 //par defaut = 2
	nRF_WriteRegister(HandleSPI,HandleGPIO,RF_SETUP,RATE_1MBPS|OUPUTPOWER_M6DB);
	nRF_WriteRegister(HandleSPI,HandleGPIO,EN_RXADDR,DATAPIPE_0);

	HandleRF.Reg[R_CONFIG] = RX_MODE|EN_CRC|NO_INTERUPTS;
	nRF_WriteRegister(HandleSPI,HandleGPIO,CONFIG,HandleRF.Reg[R_CONFIG]); 	 //update nrf

	nRF_WriteRXADDR(HandleSPI,HandleGPIO, 0x07, RX_ADDR_P0); 				 // set the Rx adresse E7
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
	//System_10msDelay(1);
	nRF_setCE(true,HandleGPIO);
}

void nRF_flushBuffers(){
	int i = 0;
	for(i = 0; i < PLAYER_BUFFER_SIZE; ++i){
		HandleRF.TXPayload[i] = 0;
		HandleRF.RXPayload[i] = 0;
	}
	// set mcu memory to zero
	for(i = 0; i < 10;i++){
		HandleRF.Reg[i] = 0;
	}
	for(i = 0; i < 10;i++){
		HandleRF.RXADDR[i] = 0;
	}
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


//for spi communication refer to wikipedia and mcu datasheet
uint16_t nRF_ReadRegister(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO,uint16_t Reg){
	uint16_t answer = 0;
	demux_connect_to(HandleRF.structCSNPin); //Set csn for spi communication
	SPI_write(HandleSPI, Reg); //register adress
	SPI_write(HandleSPI, Reg); //dummy data
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_2_Words); //wait for two words (STATUS + REG)
	HandleRF.Reg[R_STATUS] = SPI_read(HandleSPI);					//read buffer
	answer = SPI_read(HandleSPI);
	demux_disconnect();
	return answer;
}

void nRF_WriteRegister (SPI_Handle HandleSPI,GPIO_Handle HandleGPIO,uint16_t Reg,uint16_t value)
{
	demux_connect_to(HandleRF.structCSNPin);
	SPI_write(HandleSPI,W_REGISTER | Reg); //register adress
	SPI_write(HandleSPI, value << 8);
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_2_Words); //wait for status and junk
	HandleRF.Reg[R_STATUS] = SPI_read(HandleSPI);
	HandleRF.Reg[R_JUNK] = SPI_read(HandleSPI);
	demux_disconnect();
}



/**************************************************************************************************
* \fn nRF_WriteTXPayload()
* \brief Send wireless packet
*
**************************************************************************************************/
void nRF_WriteTXPayload(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI)
{
	uint8_t i=0;

	demux_connect_to(HandleRF.structCSNPin); 	// CE pin LOW


	SPI_write(HandleSPI, W_TX_PAYLOAD);
	for(i=0; i<5; i++){SPI_write(HandleSPI, HandleRF.TXPayload[i]);}	//Write tx to spi
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_1_Word);		//get status
	HandleRF.Reg[R_STATUS] = SPI_read(HandleSPI);
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_2_Words); 	//wait for 2 words and junk them
	HandleRF.Reg[R_JUNK] = SPI_read(HandleSPI);
	HandleRF.Reg[R_JUNK] = SPI_read(HandleSPI);
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_3_Words); 	//wait for the three last one
	HandleRF.Reg[R_JUNK] = SPI_read(HandleSPI);
	HandleRF.Reg[R_JUNK] = SPI_read(HandleSPI);
	HandleRF.Reg[R_JUNK] = SPI_read(HandleSPI);


	demux_disconnect();
}


/**************************************************************************************************
*  Reading paket procedur
*  for question about registers (ex STATUS, FIFOSTATUS etc.) look in nrf's datasheet
**************************************************************************************************/

Bool nRF_Listen(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI){

	nRF_setCE(false, HandleGPIO);
	if(nRF_rxPayloadReceived()){
		while(!nRF_rxFifoIsEmpty()){
			nRF_ReadRXPayload(HandleGPIO,HandleSPI,PLAYER_BUFFER_SIZE);
		}
		nRF_setCE(true, HandleGPIO);
		return true;
	}
	else{
		nRF_setCE(true, HandleGPIO);
		return false;
	}
}

bool nRF_rxFifoIsEmpty(){
	HandleRF.Reg[R_FIFOSTATUS] = nRF_ReadRegister(HandleRobot.HandleSPI,HandleRobot.HandleGPIO,FIFO_STATUS); //update FIFO status
	if(HandleRF.Reg[R_FIFOSTATUS] & 0x01){  //empty fifo bit
		return true;
	}
	else{
		return false;
	}
}

bool nRF_rxPayloadReceived(){
	HandleRF.Reg[R_STATUS] = nRF_ReadRegister(HandleRobot.HandleSPI,HandleRobot.HandleGPIO,STATUS); //update FIFO status
	if(HandleRF.Reg[R_STATUS] & 0x40){
		nRF_WriteRegister(HandleRobot.HandleSPI,HandleRobot.HandleGPIO,STATUS, 0x40); //set the flag down
		return true;
	}
	else{
		return false;
	}
}

/**************************************************************************************************
* \fn nRF_ReadRXPayload()
* \brief Receive wireless paket from nRF24L01
*
**************************************************************************************************/
void nRF_ReadRXPayload(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI,uint16_t payloadLength)
{
		uint8_t i=0;
		uint8_t j=0;
		demux_connect_to(HandleRF.structCSNPin);
		uint16_t entireBlock = payloadLength/4;  //read payload by block of 4 bytes
		uint16_t restBlock = payloadLength%4;    //modulo of an entire block

		SPI_write(HandleSPI, R_RX_PAYLOAD);
		while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_1_Word); //write and receive status byte
		HandleRF.Reg[R_STATUS] = SPI_read(HandleSPI);
		for(i = 0; i < entireBlock; ++i){								//iterate over number of block
			for(j=0; j<4; ++j){SPI_write(HandleSPI, R_RX_PAYLOAD);}		//write dummy
			for(j=0;j<4;++j){											//receive and store data
				while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_1_Word);
				HandleRF.RXPayload[j+(4*i)] = SPI_read(HandleSPI);
			}
		}
		for(j=0; j<restBlock; ++j){SPI_write(HandleSPI, R_RX_PAYLOAD);}  //handle modulo of block (1,2 or 3 bytes)
		for(i = 0; i < restBlock; ++i){
			while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_1_Word);
			HandleRF.RXPayload[i+(4*entireBlock)] = SPI_read(HandleSPI);
		}

		demux_disconnect();
																																																		// Count the received packets
}



/**************************************************************************************************
* \fn nRF_ReadTXADDR()
* \brief Reads the nRF24L01's TX_ADDR register
*
**************************************************************************************************/
void nRF_ReadTXADDR(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI)
{
	uint8_t i=0;

	demux_connect_to(HandleRF.structCSNPin);

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

	demux_disconnect();
}

void nRF_ReadRXADDR(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI, uint16_t pipe)
{
	uint8_t i=0;

	demux_connect_to(HandleRF.structCSNPin);

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

	demux_disconnect();
}





/**************************************************************************************************
* \fn nRF_WriteTXADDR(unsigned int uiID)
* \brief Write TXADDR reg en fonction de l'ID du robot voul
*
**************************************************************************************************/
void nRF_WriteTXADDR(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO, uint8_t uiID)
{
	unsigned int i=0;


	demux_connect_to(HandleRF.structCSNPin);

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

	demux_disconnect();
}

void nRF_WriteRXADDR(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO, uint8_t uiID, uint16_t pipe)
{
	unsigned int i=0;


	demux_connect_to(HandleRF.structCSNPin);

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

	demux_disconnect();
}

void nRF_PowerUp(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO, Bool ans){



	if(ans)
		HandleRF.Reg[R_CONFIG] = HandleRF.Reg[R_CONFIG] | POWER_UP;
	else
		HandleRF.Reg[R_CONFIG] = HandleRF.Reg[R_CONFIG] & ~POWER_UP;

	nRF_WriteRegister(HandleSPI,HandleGPIO,CONFIG,HandleRF.Reg[R_CONFIG]); //Power up
}










/**************************************************************************************************
* \fn nRF_FlushRX()
* \brief Flush RX FIFO
*
**************************************************************************************************/
void nRF_FlushRX(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO)
{
	demux_connect_to(HandleRF.structCSNPin);
	SPI_write(HandleSPI, FLUSH_RX);
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_1_Word); //wait
	HandleRF.Reg[R_STATUS] = SPI_read(HandleSPI);
	demux_disconnect();
}



/**************************************************************************************************
* \fn nRF_FlushTX()
* \brief Flush TX FIFO
*
**************************************************************************************************/
void nRF_FlushTX(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO)
{
	SPI_write(HandleSPI, FLUSH_TX);
	while(SPI_getRxFifoStatus(HandleSPI) < SPI_FifoStatus_1_Word); //wait
	HandleRF.Reg[R_STATUS] = SPI_read(HandleSPI);
}


void nRF_DisplayRegister(void)
{
	System_printf("Status = 0x%x \n",HandleRF.Reg[R_STATUS]);
	System_printf("Config = 0x%x \n",HandleRF.Reg[R_CONFIG]);
	System_printf("RF_Setup = 0x%x \n",HandleRF.Reg[R_RFSETUP]);
	System_printf("PLLength0 = 0x%x \n",HandleRF.Reg[R_PLLength0]);
	System_printf("PLLength1 = 0x%x \n",HandleRF.Reg[R_PLLength1]);
	System_printf("RF_CH = 0x%x \n",HandleRF.Reg[R_RFChannel]);
	System_printf("EN_RXADDR = 0x%x \n",HandleRF.Reg[R_ENRXADDR]);
	System_printf("FIFO_Status = 0x%x \n",HandleRF.Reg[R_FIFOSTATUS]);
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

	uint16_t i =0;
	System_printf("Payload = ");
	for(i = 0; i < PLAYER_BUFFER_SIZE; ++i){
		System_printf("%x",HandleRF.RXPayload[i]);
	}
	System_printf("\n\r");

}

bool nRF_test(){
	return nRF_ReadRegister(HandleRobot.HandleSPI,HandleRobot.HandleGPIO,CONFIG) == HandleRF.Reg[R_CONFIG]; //after init
}


