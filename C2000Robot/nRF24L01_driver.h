/*
 * nRF24L01_driver.h
 *
 *  Created on: 2013-10-24
 *      Author: Mathieu Garon
 */

#ifndef NRF24L01_DRIVER_H_
#define NRF24L01_DRIVER_H_

#include <stdio.h>
#include "Robocup_Define.h"

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

 static const uint16_t CONFIG          = 0x00u;
 static const uint16_t EN_AA           = 0x01u;
 static const uint16_t EN_RXADDR       = 0x02u;
 static const uint16_t SETUP_AW        = 0x03u;
 static const uint16_t SETUP_RETR      = 0x04u;
 static const uint16_t RF_CH           = 0x05u;
 static const uint16_t RF_SETUP        = 0x06u;
 static const uint16_t STATUS          = 0x07u;
 static const uint16_t OBSERVE_TX      = 0x08u;
 static const uint16_t CD              = 0x09u;
 static const uint16_t RX_ADDR_P0      = 0x0Au;
 static const uint16_t RX_ADDR_P1      = 0x0Bu;
 static const uint16_t RX_ADDR_P2      = 0x0Cu;
 static const uint16_t RX_ADDR_P3      = 0x0Du;
 static const uint16_t RX_ADDR_P4      = 0x0Eu;
 static const uint16_t RX_ADDR_P5      = 0x0Fu;
 static const uint16_t TX_ADDR         = 0x10u;
 static const uint16_t RX_PW_P0        = 0x11u;
 static const uint16_t RX_PW_P1        = 0x12u;
 static const uint16_t RX_PW_P2        = 0x13u;
 static const uint16_t RX_PW_P3        = 0x14u;
 static const uint16_t RX_PW_P4        = 0x15u;
 static const uint16_t RX_PW_P5        = 0x16u;
 static const uint16_t FIFO_STATUS     = 0x17u;
 static const uint16_t DYNPD           = 0x1Cu;
 static const uint16_t FEATURE         = 0x1Du;

/*********************************************************************
* nrf's Commands (shifted 8 bit for piccolo mcu compatibility)
*********************************************************************/
 static const uint16_t R_REGISTER     	  = 0x00u;
 static const uint16_t W_REGISTER         = 0x20u;
 static const uint16_t R_RX_PAYLOAD    	  = 0x61u;
 static const uint16_t W_TX_PAYLOAD    	  = 0xA0u;
 static const uint16_t FLUSH_TX        	  = 0xE1u;
 static const uint16_t FLUSH_RX        	  = 0xE2u;
 static const uint16_t REUSE_TX_PL        = 0xE3u;
 static const uint16_t ACTIVATE           = 0x50u;
 static const uint16_t R_RX_PL_WID        = 0x60u;
 static const uint16_t W_ACK_PAYLOAD      = 0xA8u;
 static const uint16_t W_TX_PAYLOAD_NOACK = 0x58u;
 static const uint16_t NOP                = 0xFFu;

/*********************************************************************
* nrf's SETUP
*********************************************************************/

static const uint16_t RATE_2MBPS			= 0x08u;
static const uint16_t RATE_1MBPS			= 0x00u;
static const uint16_t RATE_250KBPS		    = 0x28u;
static const uint16_t OUPUTPOWER_M18DB	    = 0x00u;
static const uint16_t OUPUTPOWER_M12DB	    = 0x02u;
static const uint16_t OUPUTPOWER_M6DB		= 0x04u;
static const uint16_t OUPUTPOWER_0DB		= 0x06u;

/*********************************************************************
* nrf's datapipes
*********************************************************************/
static const uint16_t DATAPIPE_0	= 0x01u;
static const uint16_t DATAPIPE_1	= 0x02u;
static const uint16_t DATAPIPE_2	= 0x04u;
static const uint16_t DATAPIPE_3	= 0x08u;
static const uint16_t DATAPIPE_4	= 0x10u;
static const uint16_t DATAPIPE_5	= 0x20u;

/*********************************************************************
* nrf's Config
*********************************************************************/
static const uint16_t TX_MODE					= 0x00u;
static const uint16_t RX_MODE					= 0x01u;
static const uint16_t POWER_DOWN  			    = 0x00u;
static const uint16_t POWER_UP				    = 0x02u;
static const uint16_t NO_INT_RECEIV			    = 0x40u;
static const uint16_t NO_INT_TRANSMIT			= 0x20u;
static const uint16_t NO_INT_MAXRETRANSMIT	    = 0x10u;
static const uint16_t NO_INTERUPTS			    = 0x70u;
static const uint16_t EN_CRC					= 0x08u;
/*******************************************************************************
 * User Interface Function
 *******************************************************************************/

void nRFInit(GPIO_Handle HandleGPIO, SPI_Handle HandleSPI, GPIO_Number_e CePin, chip_select MuxValue);
void nRF_flushBuffers();
Bool nRF_Listen(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI);
void nRF_DisplayRegister(void);
void nRF_DisplayRXAdress(void);
void nRF_DisplayTXAdress(void);
void nRF_DisplayRXPayload(void);

/*******************************************************************************
* Specialized function
*******************************************************************************/
void nRF_setCE(bool bEnable,GPIO_Handle HandleGPIO);
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
bool nRF_rxFifoIsEmpty();
bool nRF_rxPayloadReceived();

void nRF_WriteTXPayload(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI);
void nRF_ReadRXPayload(GPIO_Handle HandleGPIO,SPI_Handle HandleSPI,uint16_t payloadLenght);

/*******************************************************************************
* Test function
*******************************************************************************/
bool nRF_test();




#endif /* NRF24L01_DRIVER_H_ */
