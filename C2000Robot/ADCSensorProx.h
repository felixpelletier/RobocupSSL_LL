#ifndef PROXIMITY_SENSOR_DRIVER_H_
#define PROXIMITY_SENSOR_DRIVER_H_

#include "Robocup_Define.h"

static const uint16_t DIST_BALL = 2700;

interrupt void adc_isr(void);
void ADC_INIT_Fn();
void ADC_SETUP_Fn();
int Prox_isBallClose();

#endif
