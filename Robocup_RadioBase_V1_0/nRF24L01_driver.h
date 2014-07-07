/*
 * nRF24L01_driver.h
 *
 *  Created on: 2013-10-24
 *      Author: Mathieu Garon
 */

#ifndef NRF24L01_DRIVER_H_
#define NRF24L01_DRIVER_H_

#include <stdio.h>
#include "unpacker.h"
#include "Robocup_Define_RadioBase.h"

/*********************************************************************
* Enumerator for Reg[10] list in nrf's Handle
*********************************************************************/

enum  //enume pour la liste des REG dans nRF24L01_Handle
{
	R_STATUS=0,
	R_CONFIG,
	R_RFSETUP,
	R_PLLength0,
	R_PLLength1,
	R_PipeNumber,
	R_RFChannel,
	R_ENRXADDR,
	R_FIFOSTATUS,
	R_JUNK

};



/*********************************************************************
* nrf's register (shifted by 8 for compatibility with piccolo mcu)
*********************************************************************/

 #define CONFIG          0x00 << 8
 #define EN_AA           0x01 << 8
 #define EN_RXADDR       0x02 << 8
 #define SETUP_AW        0x03 << 8
 #define SETUP_RETR      0x04 << 8
 #define RF_CH           0x05 << 8
 #define RF_SETUP        0x06 << 8
 #define STATUS          0x07 << 8
 #define OBSERVE_TX      0x08 << 8
 #define CD              0x09 << 8
 #define RX_ADDR_P0      0x0A << 8
 #define RX_ADDR_P1      0x0B << 8
 #define RX_ADDR_P2      0x0C << 8
 #define RX_ADDR_P3      0x0D << 8
 #define RX_ADDR_P4      0x0E << 8
 #define RX_ADDR_P5      0x0F << 8
 #define TX_ADDR         0x10 << 8
 #define RX_PW_P0        0x11 << 8
 #define RX_PW_P1        0x12 << 8
 #define RX_PW_P2        0x13 << 8
 #define RX_PW_P3        0x14 << 8
 #define RX_PW_P4        0x15 << 8
 #define RX_PW_P5        0x16 << 8
 #define FIFO_STATUS     0x17 << 8
 #define DYNPD           0x1C << 8
 #define FEATURE         0x1D << 8

/*********************************************************************
* nrf's Commands (shifted 8 bit for piccolo mcu compatibility)
*********************************************************************/
 #define R_REGISTER     		0x00 << 8
 #define W_REGISTER      		0x20 << 8
 #define R_RX_PAYLOAD    		0x61 << 8
 #define W_TX_PAYLOAD    		0xA0 << 8
 #define FLUSH_TX        		0xE1 << 8
 #define FLUSH_RX        		0xE2 << 8
 #define REUSE_TX_PL        0xE3 << 8
 #define ACTIVATE           0x50 << 8
 #define R_RX_PL_WID        0x60 << 8
 #define W_ACK_PAYLOAD      0xA8 << 8
 #define W_TX_PAYLOAD_NOACK 0x58 << 8
 #define NOP                0xFF << 8

/*********************************************************************
* nrf's SETUP
*********************************************************************/

#define RATE_2MBPS			0X8
#define RATE_1MBPS			0X0
#define RATE_250KBPS		0X28
#define OUPUTPOWER_M18DB	0X0
#define OUPUTPOWER_M12DB	0X2
#define OUPUTPOWER_M6DB		0X4
#define OUPUTPOWER_0DB		0X6

/*********************************************************************
* nrf's datapipes
*********************************************************************/
#define DATAPIPE_0	0x01
#define DATAPIPE_1	0x02
#define DATAPIPE_2	0x04
#define DATAPIPE_3	0x08
#define DATAPIPE_4	0x10
#define DATAPIPE_5	0x20

/*********************************************************************
* nrf's Config
*********************************************************************/
#define TX_MODE					0x00
#define RX_MODE					0x01
#define POWER_DOWN  			0x00
#define POWER_UP				0x02
#define NO_INT_RECEIV			0x40
#define NO_INT_TRANSMIT			0x20
#define NO_INT_MAXRETRANSMIT	0x10
#define NO_INTERUPTS			0x70
#define EN_CRC					0x08

/*******************************************************************************
 * User Interface Function
 *******************************************************************************/

void nRFInit(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI,GPIO_Number_e CePin,GPIO_Number_e CSNPin);
Bool nRF_Listen(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI);
void nRF_DisplayRegister(void);
void nRF_DisplayRXAdress(void);
void nRF_DisplayTXAdress(void);
void nRF_DisplayRXPayload(void);
void nRF_DisplayTXPayload(void);
void nRF_DisplayRetransmit(void);

/*******************************************************************************
* Specialized function
*******************************************************************************/
void nRF_setCE(bool bEnable,GPIO_Handle HandleGPIO);
void nRF_setCSN(bool bEnable,GPIO_Handle HandleGPIO);
uint16_t nRF_ReadRegister(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO,uint16_t Reg);
void nRF_WriteRegister(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO,uint16_t Reg,uint16_t value);
void nRF_ReadTXADDR(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI);
void nRF_WriteTXADDR(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO, uint8_t uiID);
void nRF_ReadRXADDR(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI, uint16_t pipe);
void nRF_WriteRXADDR(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO, uint8_t uiID, uint16_t pipe);
void nRF_ReadENRXADDR(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI);
void nRF_WriteENRXADDR();
void nRF_FlushRX(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO);
void nRF_FlushTX(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO);
void nRF_PowerUp(SPI_Handle HandleSPI,GPIO_Handle HandleGPIO, Bool ans);

void nRF_sendPackets(UnPacker_Handle *buffers);
void nRF_resetTransmitStatus();
bool nRF_payloadSent();
bool nRF_maxRetransmit();
bool nRF_isTxFull();
void nRF_WriteTXPayload(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI,uint16_t payloadLenght);
void nRF_ReadRXPayload(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI);

bool nRF_test();




#endif /* NRF24L01_DRIVER_H_ */
