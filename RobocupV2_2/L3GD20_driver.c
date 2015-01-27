/*
 * L3GD20_driver.c
 *
 *  Created on: 2013-11-08
 *      Author: Philippe Babin
 */


#include "CS_demux.h"
#include "L3GD20_driver.h"

/**************************************************************************************************
* \fn nRFInit()
* \brief Initialisation du module L3GD20
*
**************************************************************************************************/

void gyro_init(chip_select pSCpin){
	HandleGyro.SCpin = pSCpin;


    /* Set CTRL_REG1 (0x20)
       ====================================================================
       BIT  Symbol    Description                                   Default
       ---  ------    --------------------------------------------- -------
       7-6  DR1/0     Output data rate                                   00
       5-4  BW1/0     Bandwidth selection                                00
         3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
         2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
         1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
         0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

      /* Switch to normal mode and enable all three channels */
    	//L3GD20_WriteRegister(L3GD20_REGISTER_CTRL_REG1, 0x0F);
    gyro_writeRegister(L3GD20_REGISTER_CTRL_REG1, 0x0C);

    /* Set CTRL_REG4 (0x23)
       ====================================================================
       BIT  Symbol    Description                                   Default
       ---  ------    --------------------------------------------- -------
         7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
         6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
       5-4  FS1/0     Full scale selection                               00
                                      00 = 250 dps
                                      01 = 500 dps
                                      10 = 2000 dps
                                      11 = 2000 dps
         0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */


    gyro_writeRegister(L3GD20_REGISTER_CTRL_REG4, 0x10);

   // System_printf("Reponse :0x%x",L3GD20_ReadRegister(HandleRobot.HandleSPI, HandleRobot.HandleGPIO, L3GD20_REGISTER_WHO_AM_I));
}

uint16_t gyro_readRegister( uint16_t pReg){
	uint16_t lAnswer = 0;
	demux_connect_to(HandleGyro.SCpin);

	SPI_write(HandleRobot.HandleSPI, pReg | 0x8000); // Set to read
	SPI_write(HandleRobot.HandleSPI, 0xFFFF); //dummy data
	while(SPI_getRxFifoStatus(HandleRobot.HandleSPI) < SPI_FifoStatus_2_Words); //wait for two words (STATUS + REG)
	SPI_read(HandleRobot.HandleSPI);
	lAnswer = SPI_read(HandleRobot.HandleSPI);

	demux_disconnect();
	return lAnswer & 0x00FF; // Set 0 the first 2 bytes
}

void gyro_writeRegister(uint16_t pReg, uint16_t pValue){
	demux_connect_to(HandleGyro.SCpin);

	SPI_write(HandleRobot.HandleSPI, pReg); //put a 0 on the first
	SPI_write(HandleRobot.HandleSPI, pValue << 8);
	while(SPI_getRxFifoStatus(HandleRobot.HandleSPI) < SPI_FifoStatus_2_Words); //wait for two words (STATUS + REG)
	SPI_read(HandleRobot.HandleSPI);
	SPI_read(HandleRobot.HandleSPI);

	demux_disconnect();
}

void gyro_readZData(){
	uint16_t lH,lL;
	lL = gyro_readRegister(L3GD20_REGISTER_OUT_Z_L);
	lH = gyro_readRegister(L3GD20_REGISTER_OUT_Z_H);

	HandleGyro.outZ = ((lH<<8)| lL);

	//System_printf("%x %x\r\n", l, h);
	System_printf("%d\r\n", HandleGyro.outZ);

}

int gyro_test(){
	return gyro_readRegister(L3GD20_REGISTER_WHO_AM_I) == L3GD20_ID;

}
