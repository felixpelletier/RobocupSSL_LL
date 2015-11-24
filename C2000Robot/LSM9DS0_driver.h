/*
 * LSM9DS0_driver.h
 *
 *  Created on: 2013-11-08
 *      Author: Philippe Babin
 */

#ifndef LSM9DS0_DRIVER_H_
#define LSM9DS0_DRIVER_H_

#include <stdio.h>
#include "Robocup_Define.h"

/*********************************************************************
* LSM9DS0's register (shifted by 8 for compatibility with piccolo mcu)
*********************************************************************/
#define LSM9DS0_ID_G              				  0x00D4
#define LSM9DS0_ID_XM              				  0x0049

#define	 LSM9DS0_REGISTER_WHO_AM_I_G              0x0F << 8 // 11010100   r
#define LSM9DS0_REGISTER_CTRL_REG1_G             0x20 << 8 // 00000111   rw
#define LSM9DS0_REGISTER_CTRL_REG2_G             0x21 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_CTRL_REG3_G             0x22 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_CTRL_REG4_G             0x23 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_CTRL_REG5_G             0x24 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_REFERENCE_G             0x25 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_STATUS_REG_G            0x27 << 8 //            r
#define LSM9DS0_REGISTER_OUT_X_L_G               0x28 << 8 //            r
#define LSM9DS0_REGISTER_OUT_X_H_G               0x29 << 8 //            r
#define LSM9DS0_REGISTER_OUT_Y_L_G               0x2A << 8 //            r
#define LSM9DS0_REGISTER_OUT_Y_H_G               0x2B << 8 //            r
#define LSM9DS0_REGISTER_OUT_Z_L_G               0x2C << 8 //            r
#define LSM9DS0_REGISTER_OUT_Z_H_G               0x2D << 8 //            r
#define LSM9DS0_REGISTER_FIFO_CTRL_REG_G         0x2E << 8 // 00000000   rw
#define LSM9DS0_REGISTER_FIFO_SRC_REG_G          0x2F << 8 //            r
#define LSM9DS0_REGISTER_INT1_CFG_G              0x30 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_INT1_SRC_G              0x31 << 8 //            r
#define LSM9DS0_REGISTER_TSH_XH_G                0x32 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_TSH_XL_G                0x33 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_TSH_YH_G                0x34 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_TSH_YL_G                0x35 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_TSH_ZH_G                0x36 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_TSH_ZL_G                0x37 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_INT1_DURATION_G         0x38 << 8 // 00000000   rw

// Magneto
#define LSM9DS0_REGISTER_OUT_TEMP_L_XM           0x05 << 8 //            r
#define LSM9DS0_REGISTER_OUT_TEMP_H_XM           0x06 << 8 //            r
#define LSM9DS0_REGISTER_STATUS_REG_M            0x07 << 8 //            r
#define LSM9DS0_REGISTER_OUT_X_L_M               0x08 << 8 //            r
#define LSM9DS0_REGISTER_OUT_X_H_M               0x09 << 8 //            r
#define LSM9DS0_REGISTER_OUT_Y_L_M               0x0A << 8 //            r
#define LSM9DS0_REGISTER_OUT_Y_H_M               0x0B << 8 //            r
#define LSM9DS0_REGISTER_OUT_Z_L_M               0x0C << 8 //            r
#define LSM9DS0_REGISTER_OUT_Z_H_M               0x0D << 8 //            r
#define	 LSM9DS0_REGISTER_WHO_AM_I_XM             0x0F << 8 // 01001001   r
#define LSM9DS0_REGISTER_INT_CTRL_REG_M          0x12 << 8 //            r
#define LSM9DS0_REGISTER_INT_SRC_REG_M           0x13 << 8 //            rw
#define LSM9DS0_REGISTER_INT_THS_L_M             0x14 << 8 //            rw
#define LSM9DS0_REGISTER_INT_THS_H_M             0x15 << 8 //            rw
#define LSM9DS0_REGISTER_OFFSET_X_L_M            0x16 << 8 //            rw
#define LSM9DS0_REGISTER_OFFSET_X_H_M            0x17 << 8 //            rw
#define LSM9DS0_REGISTER_OFFSET_Y_L_M            0x18 << 8 //            rw
#define LSM9DS0_REGISTER_OFFSET_Y_H_M            0x19 << 8 //            rw
#define LSM9DS0_REGISTER_OFFSET_Z_L_M            0x1A << 8 //            rw
#define LSM9DS0_REGISTER_OFFSET_Z_H_M            0x1B << 8 //            rw
#define LSM9DS0_REGISTER_REFERENCE_X             0x1C << 8 // 00000000   rw
#define LSM9DS0_REGISTER_REFERENCE_Y             0x1D << 8 // 00000000   rw
#define LSM9DS0_REGISTER_REFERENCE_Z             0x1E << 8 // 00000000   rw
#define LSM9DS0_REGISTER_REG0_XM                 0x1F << 8 // 00000000   rw
#define LSM9DS0_REGISTER_REG1_XM             	  0x20 << 8 // 00000111   rw
#define LSM9DS0_REGISTER_REG2_XM             	  0x21 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_REG3_XM             	  0x22 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_REG4_XM             	  0x23 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_REG5_XM             	  0x24 << 8 // 00011000   rw
#define LSM9DS0_REGISTER_REG6_XM             	  0x25 << 8 // 00100000   rw
#define LSM9DS0_REGISTER_REG7_XM             	  0x26 << 8 // 00000001   rw


// Accelero
#define LSM9DS0_REGISTER_STATUS_REG_A            0x27 << 8 //            r
#define LSM9DS0_REGISTER_OUT_X_L_A               0x28 << 8 //            r
#define LSM9DS0_REGISTER_OUT_X_H_A               0x29 << 8 //            r
#define LSM9DS0_REGISTER_OUT_Y_L_A               0x2A << 8 //            r
#define LSM9DS0_REGISTER_OUT_Y_H_A               0x2B << 8 //            r
#define LSM9DS0_REGISTER_OUT_Z_L_A               0x2C << 8 //            r
#define LSM9DS0_REGISTER_OUT_Z_H_A               0x2D << 8 //            r
#define LSM9DS0_REGISTER_FIFO_CTRL_REG           0x2E << 8 // 00000000   rw
#define LSM9DS0_REGISTER_FIFO_SRC_REG            0x2F << 8 //            r
#define LSM9DS0_REGISTER_INT_GEN_1_REG           0x30 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_INT_GEN_1_SRC           0x31 << 8 //            r
#define LSM9DS0_REGISTER_INT_GEN_1_THS           0x32 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_INT_GEN_1_DURATION      0x33 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_INT_GEN_2_REG           0x34 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_INT_GEN_2_SRC           0x35 << 8 //            r
#define LSM9DS0_REGISTER_INT_GEN_2_THS           0x36 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_INT_GEN_2_DURATION      0x37 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_CLICK_CFG    			  0x38 << 8 // 00000000   rw
#define LSM9DS0_REGISTER_CLICK_SRC		          0x39 << 8 //            r
#define LSM9DS0_REGISTER_CLICK_THS    			  0x3A << 8 // 00000000   rw
#define LSM9DS0_REGISTER_TIME_LIMIT    			  0x3B << 8 // 00000000   rw
#define LSM9DS0_REGISTER_TIME_LATENCY    		  0x3C << 8 // 00000000   rw
#define LSM9DS0_REGISTER_TIME_WINDOW       		  0x3D << 8 // 00000000   rw
#define LSM9DS0_REGISTER_Act_THS    			  0x3E << 8 // 00000000   rw
#define LSM9DS0_REGISTER_Act_DUR    			  0x3F << 8 // 00000000   rw



/*******************************************************************************
 * User Interface Function
 *******************************************************************************/
void imu_init(chip_select pGyroSCpin, chip_select pAccSCpin);

uint16_t imu_readRegister( uint16_t pReg, chip_select pSCpin);
void imu_writeRegister(uint16_t pReg, uint16_t pValue, chip_select pSCpin);
uint16_t imu_readRegister_G( uint16_t pReg);
void imu_writeRegister_G(uint16_t pReg, uint16_t pValue);
uint16_t imu_readRegister_AM( uint16_t pReg);
void imu_writeRegister_AM(uint16_t pReg, uint16_t pValue);
void imu_readZData();
void imu_readXYZAccero();

int imu_test();


#endif /* LSM9DS0_DRIVER_H_ */
