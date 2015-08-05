/*
 * quad_driver.c
 *
 *  Created on: 2013-11-08
 *      Author: Mathieu Garon
 */


#include "quad_driver.h"
#include "SPI.h"

quad_Handle quad_init(chip_select pCSNPin){
	quad_Handle lHandle;
	lHandle.structCSNPin = pCSNPin;
    quad_WriteRegister(QUAD_CONFIG0, 0x07, &lHandle);  //counter0 = 16 bits and counter1 = 16 bit (no counter2)
    quad_WriteRegister(QUAD_CONFIG1, 0x80, &lHandle);  // counter TTL
    return lHandle;
}

uint16_t quad_readRegister(uint16_t pReg,quad_Handle *pQuad){
	uint16_t answer = 0;
	demux_connect_to(pQuad->structCSNPin);
	SPI_write_8bits(HandleRobot.HandleSPI, pReg | QUAD_READ); //put a 1 on the first
	SPI_write_8bits(HandleRobot.HandleSPI, pReg); //dummy data
	while(SPI_getRxFifoStatus(HandleRobot.HandleSPI) < SPI_FifoStatus_2_Words); //wait for two words (STATUS + REG)
	answer = SPI_read(HandleRobot.HandleSPI);
	answer = SPI_read(HandleRobot.HandleSPI);
	demux_disconnect();
	return answer;
}

void quad_WriteRegister(uint16_t pReg, uint16_t pValue,quad_Handle *pQuad){
	demux_connect_to(pQuad->structCSNPin);
	SPI_write_8bits(HandleRobot.HandleSPI, pReg & QUAD_WRITE); //put a 0 on the first
	SPI_write_8bits(HandleRobot.HandleSPI, pValue); //dummy data
	while(SPI_getRxFifoStatus(HandleRobot.HandleSPI) < SPI_FifoStatus_2_Words); //wait for two words (STATUS + REG)
	SPI_read(HandleRobot.HandleSPI);
	SPI_read(HandleRobot.HandleSPI);
	demux_disconnect();
}


void quad_readCounters(quad_Handle *pQuad){
	uint16_t lCount1[2] = {0,0};
	uint16_t lCount0[2] = {0,0};

	demux_connect_to(pQuad->structCSNPin);
	SPI_write_8bits(HandleRobot.HandleSPI, QUAD_COUNTER | QUAD_READ); //put a 0 on the first
	SPI_write(HandleRobot.HandleSPI, 0x00); //dummy
	SPI_write(HandleRobot.HandleSPI, 0x00); //dummy
	SPI_write(HandleRobot.HandleSPI, 0x00); //dummy
	while(SPI_getRxFifoStatus(HandleRobot.HandleSPI) < SPI_FifoStatus_4_Words); //wait for two words (STATUS + REG)
	SPI_read(HandleRobot.HandleSPI);
	SPI_read(HandleRobot.HandleSPI);
	SPI_read(HandleRobot.HandleSPI);
	lCount1[1] = SPI_read(HandleRobot.HandleSPI);
	SPI_write(HandleRobot.HandleSPI, 0x00); //dummy
	SPI_write(HandleRobot.HandleSPI, 0x00); //dummy
	SPI_write(HandleRobot.HandleSPI, 0x00); //dummy
	while(SPI_getRxFifoStatus(HandleRobot.HandleSPI) < SPI_FifoStatus_3_Words);
	lCount1[0] = SPI_read(HandleRobot.HandleSPI);
	lCount0[1] = SPI_read(HandleRobot.HandleSPI);
	lCount0[0] = SPI_read(HandleRobot.HandleSPI);

	demux_disconnect();
	pQuad->Count0 = (lCount0[1] << 8) | lCount0[0];
	pQuad->Count1 = (lCount1[1] << 8) | lCount1[0];

	quad_WriteRegister( QUAD_CTRL, QUAD_RESETCNT0 | QUAD_RESETCNT1 | QUAD_RESETCNT2,pQuad); //reset counters

	quad_calculateSpeed(pQuad);
}

void quad_calculateSpeed(quad_Handle *pQuad){
	_iq lDistance0 = _IQ(pQuad->Count0);
	_iq lDistance1 = _IQ(pQuad->Count1);
	pQuad->wheelVelocity[0] = _IQmpy(lDistance0,HandleRobot.robotParam.speedFactor);
	pQuad->wheelVelocity[1] = _IQmpy(lDistance1,HandleRobot.robotParam.speedFactor);
}

void quad_displayCounters(quad_Handle *pQuad){
	System_printf("count0 =%d count1 =%d\n\r",pQuad->Count0,pQuad->Count1);
}

void quad_displayVelocity(quad_Handle *pQuad){
	System_printf("wheel0 =%f wheel1 =%f cm/10ms\n\r",_IQtoF(pQuad->wheelVelocity[0]),
			_IQtoF(pQuad->wheelVelocity[1]));

}

bool quad_test(quad_Handle *pQuad){
	return quad_readRegister(QUAD_CONFIG1, pQuad) == 0x80;  // if initialised before!
}
