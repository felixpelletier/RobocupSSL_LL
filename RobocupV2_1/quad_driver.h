/*
 * quad_driver.h
 *
 *  Created on: 2013-11-08
 *      Author: Mathieu Garon
 */

#ifndef QUAD_DRIVER_H_
#define QUAD_DRIVER_H_

#include <stdio.h>
#include "Robocup_Define.h"
#include "CS_demux.h"

/****************************************************************************
 *   MD quadrature decoder Register
 *****************************************************************************/
#define QUAD_CONFIG0 0x00 <<8
#define QUAD_CONFIG1 0x01 <<8
#define QUAD_CONFIG2 0x02 <<8
#define QUAD_CONFIG3 0x03 <<8
#define QUAD_CONFIG4 0x04 <<8
#define QUAD_CTRL 	 0x30 <<8    //mainly used to set counter to zero
#define QUAD_COUNTER 0x08 <<8
/****************************************************************************
 *   MD quadrature decoder Command
 *****************************************************************************/
#define QUAD_RESETCNT0 0x01 <<8  // used with CTRL register to reset counter
#define QUAD_RESETCNT1 0x02 <<8
#define QUAD_RESETCNT2 0x04 <<8
#define QUAD_READ	   0x80 <<8
#define QUAD_WRITE	   ~0x80 <<8 //modification needed



quad_Handle quad_init(chip_select pCSNPin);
uint16_t quad_readRegister(uint16_t pReg,quad_Handle *pQuad);
void quad_WriteRegister(uint16_t pReg, uint16_t pValue,quad_Handle *pQuad);
void quad_readCounters(quad_Handle *pQuad);
void quad_calculateSpeed(quad_Handle *pQuad);

void quad_displayCounters(quad_Handle *pQuad);
void quad_displayVelocity(quad_Handle *pQuad);

bool quad_test(quad_Handle *pQuad);

#endif /* QUAD_DRIVER_H_ */
