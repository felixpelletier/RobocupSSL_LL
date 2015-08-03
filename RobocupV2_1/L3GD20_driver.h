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
static const uint16_t	L3GD20_ID              			    = 0x00D4;
static const uint16_t	L3GD20_REGISTER_WHO_AM_I            = 0x0Fu; // 11010100   r
static const uint16_t L3GD20_REGISTER_CTRL_REG1             = 0x20u; // 00000111   rw
static const uint16_t L3GD20_REGISTER_CTRL_REG2             = 0x21u; // 00000000   rw
static const uint16_t L3GD20_REGISTER_CTRL_REG3             = 0x22u; // 00000000   rw
static const uint16_t L3GD20_REGISTER_CTRL_REG4             = 0x23u; // 00000000   rw
static const uint16_t L3GD20_REGISTER_CTRL_REG5             = 0x24u; // 00000000   rw
static const uint16_t L3GD20_REGISTER_REFERENCE             = 0x25u; // 00000000   rw
static const uint16_t L3GD20_REGISTER_OUT_TEMP              = 0x26u; //            r
static const uint16_t L3GD20_REGISTER_STATUS_REG            = 0x27u; //            r
static const uint16_t L3GD20_REGISTER_OUT_X_L               = 0x28u; //            r
static const uint16_t L3GD20_REGISTER_OUT_X_H               = 0x29u; //            r
static const uint16_t L3GD20_REGISTER_OUT_Y_L               = 0x2Au; //            r
static const uint16_t L3GD20_REGISTER_OUT_Y_H               = 0x2Bu; //            r
static const uint16_t L3GD20_REGISTER_OUT_Z_L               = 0x2Cu; //            r
static const uint16_t L3GD20_REGISTER_OUT_Z_H               = 0x2Du; //            r
static const uint16_t L3GD20_REGISTER_FIFO_CTRL_REG         = 0x2Eu; // 00000000   rw
static const uint16_t L3GD20_REGISTER_FIFO_SRC_REG          = 0x2Fu; //            r
static const uint16_t L3GD20_REGISTER_INT1_CFG              = 0x30u; // 00000000   rw
static const uint16_t L3GD20_REGISTER_INT1_SRC              = 0x31u; //            r
static const uint16_t L3GD20_REGISTER_TSH_XH                = 0x32u; // 00000000   rw
static const uint16_t L3GD20_REGISTER_TSH_XL                = 0x33u; // 00000000   rw
static const uint16_t L3GD20_REGISTER_TSH_YH                = 0x34u; // 00000000   rw
static const uint16_t L3GD20_REGISTER_TSH_YL                = 0x35u; // 00000000   rw
static const uint16_t L3GD20_REGISTER_TSH_ZH                = 0x36u; // 00000000   rw
static const uint16_t L3GD20_REGISTER_TSH_ZL                = 0x37u; // 00000000   rw
static const uint16_t L3GD20_REGISTER_INT1_DURATION         = 0x38u; // 00000000   rw



/*******************************************************************************
 * User Interface Function
 *******************************************************************************/

void gyro_init(chip_select pSCpin);
uint16_t gyro_readRegister( uint16_t pReg);
void gyro_writeRegister(uint16_t pReg, uint16_t pValue);
void gyro_readZData();
int gyro_test();



#endif /* L3GD20_DRIVER_H_ */
