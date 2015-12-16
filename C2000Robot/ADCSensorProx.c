/* =================================================================================
File name: ADCSensorProx.c
Originator: Fr�d�ric St-Pierre, Annouar Hannafi
Description: Data and macro definitions for PID controller
=====================================================================================
History:
-------------------------------------------------------------------------------------
17-02-2015 Version 1.0
-------------------------------------------------------------------------------------
*/
//#include "pid.h"
#include "ADCSensorProx.h"

//start ADC
interrupt void adc_isr(void)
{
   //discard ADCRESULT0 as part of the workaround to the 1st sample errata for rev0
   //Digital_Result = ADC_readResult(HandleRobot.HandleADC, ADC_ResultNumber_0);
   ADC_clearIntFlag(HandleRobot.HandleADC, ADC_IntNumber_1);   // Clear ADCINT1 flag reinitialize for next SOC
   PIE_clearInt(HandleRobot.HandlePIE, PIE_GroupNumber_10);// Acknowledge interrupt to PIE
   return;
}

void ADC_INIT_Fn()
{
	ADC_enableBandGap(HandleRobot.HandleADC);
	ADC_enableRefBuffers(HandleRobot.HandleADC);
	ADC_powerUp(HandleRobot.HandleADC);
	ADC_enable(HandleRobot.HandleADC);
	ADC_setVoltRefSrc(HandleRobot.HandleADC, ADC_VoltageRefSrc_Int);
}

void ADC_SETUP_Fn()
{
	PIE_registerPieIntHandler(HandleRobot.HandlePIE, PIE_GroupNumber_10, PIE_SubGroupNumber_1, (intVec_t)&adc_isr);
	PIE_enableAdcInt(HandleRobot.HandlePIE, ADC_IntNumber_1); // Enable ADCINT1 in PIE
	//Note: Channel ADCINA1  will be double sampled to workaround the ADC 1st sample issue for rev0 silicon errata
	ADC_setIntPulseGenMode(HandleRobot.HandleADC, ADC_IntPulseGenMode_Prior);               //ADCINT1 trips after AdcResults latch
	ADC_enableInt(HandleRobot.HandleADC, ADC_IntNumber_1);                                  //Enabled ADCINT1
	ADC_setIntMode(HandleRobot.HandleADC, ADC_IntNumber_1, ADC_IntMode_ClearFlag);          //Disable ADCINT1 Continuous mode
	ADC_setIntSrc(HandleRobot.HandleADC, ADC_IntNumber_1, ADC_IntSrc_EOC0);                 //setup EOC0 to trigger ADCINT1 to fire
	ADC_setSocChanNumber (HandleRobot.HandleADC, ADC_SocNumber_0, ADC_SocChanNumber_A0);    //set SOC0 channel select to ADCINA0
	ADC_setSocChanNumber (HandleRobot.HandleADC, ADC_SocNumber_1, ADC_SocChanNumber_A1);    //set SOC1 channel select to ADCINA1
	ADC_setSocTrigSrc(HandleRobot.HandleADC, ADC_SocNumber_0, ADC_SocTrigSrc_Sw);     //set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
	ADC_setSocTrigSrc(HandleRobot.HandleADC, ADC_SocNumber_1, ADC_SocTrigSrc_Sw);     //set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
	ADC_setSocSampleWindow(HandleRobot.HandleADC, ADC_SocNumber_0, ADC_SocSampleWindow_7_cycles);   //set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	ADC_setSocSampleWindow(HandleRobot.HandleADC, ADC_SocNumber_1, ADC_SocSampleWindow_7_cycles);   //set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)

}

int Prox_isBallClose()
{
	uint16_t Digital_Result0 =7;
	uint16_t Digital_Result1 =7;
	ADC_forceConversion(HandleRobot.HandleADC , ADC_SocNumber_0);
		ADC_forceConversion(HandleRobot.HandleADC , ADC_SocNumber_1);
		Digital_Result0 = ADC_readResult(HandleRobot.HandleADC, ADC_ResultNumber_0); //drib_led1
		Digital_Result1 = ADC_readResult(HandleRobot.HandleADC, ADC_ResultNumber_1); //drib_led2
		//System_printf("ADC0: %8d ", Digital_Result0);
		//System_printf("ADC1: %8d\r\n", Digital_Result1);
		return Digital_Result0 < DIST_BALL || Digital_Result1 < DIST_BALL;
}
//stop ADC
