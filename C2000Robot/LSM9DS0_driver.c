/*
 * L3GD20_driver.c
 *
 *  Created on: 2013-11-08
 *      Author: Philippe Babin
 */


#include "CS_demux.h"
#include "LSM9DS0_driver.h"

/**************************************************************************************************
* \fn nRFInit()
* \brief Initialisation du module L3GD20
*
**************************************************************************************************/

void imu_init(chip_select pGyroSCpin, chip_select pAccSCpin){
	HandleImu.GyroSCpin = pGyroSCpin;
	HandleImu.AccSCpin = pAccSCpin;

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
    	//imu_writeRegister_G(LSM9DS0_REGISTER_CTRL_REG1_G, 0x07);
    imu_writeRegister_G(LSM9DS0_REGISTER_CTRL_REG1_G, 0x0C);

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


    imu_writeRegister_G(LSM9DS0_REGISTER_CTRL_REG4_G, 0x10);


    imu_writeRegister_AM(LSM9DS0_REGISTER_REG1_XM, 0x67);
    imu_writeRegister_AM(LSM9DS0_REGISTER_REG5_XM, 0xF0);
}


uint16_t imu_readRegister( uint16_t pReg, chip_select pSCpin){
	uint16_t lAnswer = 0;
	demux_connect_to(pSCpin);

	SPI_write(HandleRobot.HandleSPI, pReg | 0x8000); // Set to read
	SPI_write(HandleRobot.HandleSPI, 0xFFFF); //dummy data
	while(SPI_getRxFifoStatus(HandleRobot.HandleSPI) < SPI_FifoStatus_2_Words); //wait for two words (STATUS + REG)
	SPI_read(HandleRobot.HandleSPI);
	lAnswer = SPI_read(HandleRobot.HandleSPI);

	demux_disconnect();
	return lAnswer & 0x00FF; // Set 0 the first 2 bytes
}

void  imu_writeRegister(uint16_t pReg, uint16_t pValue, chip_select pSCpin){
	demux_connect_to(pSCpin);

	SPI_write(HandleRobot.HandleSPI, pReg); //put a 0 on the first
	SPI_write(HandleRobot.HandleSPI, pValue << 8);
	while(SPI_getRxFifoStatus(HandleRobot.HandleSPI) < SPI_FifoStatus_2_Words); //wait for two words (STATUS + REG)
	SPI_read(HandleRobot.HandleSPI);
	SPI_read(HandleRobot.HandleSPI);

	demux_disconnect();
}

uint16_t imu_readRegister_G( uint16_t pReg){
	return imu_readRegister(pReg, HandleImu.GyroSCpin);
}
void imu_writeRegister_G(uint16_t pReg, uint16_t pValue){
	imu_writeRegister(pReg, pValue, HandleImu.GyroSCpin);
}

uint16_t imu_readRegister_AM( uint16_t pReg){
	return imu_readRegister(pReg, HandleImu.AccSCpin);
}

void imu_writeRegister_AM(uint16_t pReg, uint16_t pValue){
	imu_writeRegister(pReg, pValue, HandleImu.AccSCpin);
}

void imu_readXYZAccero(){
	uint16_t lH,lL, accX, accY, accZ;
	lL = imu_readRegister_AM(LSM9DS0_REGISTER_OUT_X_L_A);
	lH = imu_readRegister_AM(LSM9DS0_REGISTER_OUT_X_H_A);

	accX = ((lH<<8)| lL);

	lL = imu_readRegister_AM(LSM9DS0_REGISTER_OUT_Y_L_A);
	lH = imu_readRegister_AM(LSM9DS0_REGISTER_OUT_Y_H_A);

	accY = ((lH<<8)| lL);

	lL = imu_readRegister_AM(LSM9DS0_REGISTER_OUT_Z_L_A);
	lH = imu_readRegister_AM(LSM9DS0_REGISTER_OUT_Z_H_A);

	accZ = ((lH<<8)| lL);
	System_printf("x%d y%d z%d\r\n", accX, accY, accZ);//
}
void imu_readTempSensor(){
	uint16_t lH,lL, temp;
	lL = imu_readRegister_AM(LSM9DS0_REGISTER_OUT_TEMP_L_XM);
	lH = imu_readRegister_AM(LSM9DS0_REGISTER_OUT_TEMP_H_XM);

	temp = ((lH<<8)| lL);
	System_printf("temp:%d\r\n", temp);
}

void imu_readZGyro(){
	uint16_t lH,lL;
	lL = imu_readRegister_G(LSM9DS0_REGISTER_OUT_Z_L_G);
	lH = imu_readRegister_G(LSM9DS0_REGISTER_OUT_Z_H_G);

	uint16_t gyroZ = ((lH<<8)| lL);

	System_printf("%d\r\n", gyroZ);
}


int imu_test(){
	return imu_readRegister_G(LSM9DS0_REGISTER_WHO_AM_I_G) == LSM9DS0_ID_G &&
			imu_readRegister_AM(LSM9DS0_REGISTER_WHO_AM_I_XM) == LSM9DS0_ID_XM;

}
