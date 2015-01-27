/*
 * L3GD20_driver.h
 *
 *  Created on: 2013-11-08
 *      Author: Philippe Babin
 */

#ifndef L3GD20_DRIVER_H_
#define L3GD20_DRIVER_H_

#include <stdio.h>
#include "Robocup_Define.h"

/*********************************************************************
* L3GD20's register (shifted by 8 for compatibility with piccolo mcu)
*********************************************************************/
#define		L3GD20_ID              				  0x00D4
#define		L3GD20_REGISTER_WHO_AM_I              0x0F << 8 // 11010100   r
#define L3GD20_REGISTER_CTRL_REG1             0x20 << 8 // 00000111   rw
#define L3GD20_REGISTER_CTRL_REG2             0x21 << 8 // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG3             0x22 << 8 // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG4             0x23 << 8 // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG5             0x24 << 8 // 00000000   rw
#define L3GD20_REGISTER_REFERENCE             0x25 << 8 // 00000000   rw
#define L3GD20_REGISTER_OUT_TEMP              0x26 << 8 //            r
#define L3GD20_REGISTER_STATUS_REG            0x27 << 8 //            r
#define L3GD20_REGISTER_OUT_X_L               0x28 << 8 //            r
#define L3GD20_REGISTER_OUT_X_H               0x29 << 8 //            r
#define L3GD20_REGISTER_OUT_Y_L               0x2A << 8 //            r
#define L3GD20_REGISTER_OUT_Y_H               0x2B << 8 //            r
#define L3GD20_REGISTER_OUT_Z_L               0x2C << 8 //            r
#define L3GD20_REGISTER_OUT_Z_H               0x2D << 8 //            r
#define L3GD20_REGISTER_FIFO_CTRL_REG         0x2E << 8 // 00000000   rw
#define L3GD20_REGISTER_FIFO_SRC_REG          0x2F << 8 //            r
#define L3GD20_REGISTER_INT1_CFG              0x30 << 8 // 00000000   rw
#define L3GD20_REGISTER_INT1_SRC              0x31 << 8 //            r
#define L3GD20_REGISTER_TSH_XH                0x32 << 8 // 00000000   rw
#define L3GD20_REGISTER_TSH_XL                0x33 << 8 // 00000000   rw
#define L3GD20_REGISTER_TSH_YH                0x34 << 8 // 00000000   rw
#define L3GD20_REGISTER_TSH_YL                0x35 << 8 // 00000000   rw
#define L3GD20_REGISTER_TSH_ZH                0x36 << 8 // 00000000   rw
#define L3GD20_REGISTER_TSH_ZL                0x37 << 8 // 00000000   rw
#define L3GD20_REGISTER_INT1_DURATION         0x38 << 8   // 00000000   rw



/*******************************************************************************
 * User Interface Function
 *******************************************************************************/

void gyro_init(chip_select pSCpin);
uint16_t gyro_readRegister( uint16_t pReg);
void gyro_writeRegister(uint16_t pReg, uint16_t pValue);
void gyro_readZData();
int gyro_test();



#endif /* L3GD20_DRIVER_H_ */
