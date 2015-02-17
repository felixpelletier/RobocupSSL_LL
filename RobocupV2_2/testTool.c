/*
 * testTool.c
 *
 *  Created on: 2014-03-29
 *      Author: Mathieu Garon
 */

#include "testTool.h"

void test_is_SpiDevices_Working_Well(){
	//System_printf("%d\r\n",arduino_Test(),ARDUINO_RECALL_MESSAGE);
	bool tested = true;
	System_printf("*******SPI Device testing********\r\n");
	if(arduino_Test())
		System_printf("Arduino [OK]\r\n");
	else{
		System_printf("Arduino [FAIL]\r\n");
		tested =  false;
	}
	System_flush();

	/*
	if(gyro_test())
		System_printf("Gyro [OK]\r\n");
	else{
		System_printf("Gyro [FAIL]\r\n");
		tested = false;
	}*/

	///*
	if(imu_test())
		System_printf("Imu [OK]\r\n");
	else{
		System_printf("Imu [FAIL]\r\n");
		tested = false;
	}
	System_flush();
	//*/

	if(quad_test(&HandleRobot.HandleQuad[0]))
		System_printf("Quad 0 [OK]\r\n");
	else{
		System_printf("Quad 0 [FAIL]\r\n");
		tested = false;
	}

	System_flush();

	if(quad_test(&HandleRobot.HandleQuad[1]))
			System_printf("Quad 1 [OK]\r\n");
	else{
		System_printf("Quad 1 [FAIL]\r\n");
		tested = false;
	}

	if(nRF_test())
		System_printf("RF [OK]\r\n");
	else{
		System_printf("RF [FAIL]\r\n");
		tested = false;
	}

	System_flush();

	if(tested)
		System_printf("SUCCESS\r\n");
	else
		System_printf("FAILURE\r\n");

	System_flush();
}

void test_is_Wheel_System_Working_Well(){
	bool lSuccess = true;
	System_printf("*******Wheel testing********\r\n");

	//flush encodeur
	quad_readCounters(&HandleRobot.HandleQuad[0]);
	quad_readCounters(&HandleRobot.HandleQuad[1]);

	//MotorRotation
	dcMotor_setPWM(&HandleRobot.HandleMotor[0],900);
	dcMotor_setPWM(&HandleRobot.HandleMotor[1],900);
	dcMotor_setPWM(&HandleRobot.HandleMotor[2],900);
	dcMotor_setPWM(&HandleRobot.HandleMotor[3],900);

	test_wait();

	dcMotor_setPWM(&HandleRobot.HandleMotor[0],EPWM_BRAKE);
	dcMotor_setPWM(&HandleRobot.HandleMotor[1],EPWM_BRAKE);
	dcMotor_setPWM(&HandleRobot.HandleMotor[2],EPWM_BRAKE);
	dcMotor_setPWM(&HandleRobot.HandleMotor[3],EPWM_BRAKE);

	test_wait();

	quad_readCounters(&HandleRobot.HandleQuad[0]);
	quad_readCounters(&HandleRobot.HandleQuad[1]);

	if(HandleRobot.HandleQuad[0].Count0 != 0){
		System_printf("Wheel 0 [OK]\r\n");
	}
	else{
		System_printf("Wheel 0 [FAIL]\r\n");
		lSuccess = false;
	}

	if(HandleRobot.HandleQuad[0].Count1 != 0){
		System_printf("Wheel 1 [OK]\r\n");
	}
	else{
		System_printf("Wheel 1 [FAIL]\r\n");
		lSuccess = false;
	}

	if(HandleRobot.HandleQuad[1].Count0 != 0){
		System_printf("Wheel 2 [OK]\r\n");
	}
	else{
		System_printf("Wheel 2 [FAIL]\r\n");
		lSuccess = false;
	}

	if(HandleRobot.HandleQuad[1].Count1 != 0){
		System_printf("Wheel 3 [OK]\r\n");
	}
	else{
		System_printf("Wheel 3 [FAIL]\r\n");
		lSuccess = false;
	}

	System_flush();

	if(lSuccess){
		System_printf("[SUCCESS]\r\n");
	}
	else{
		System_printf("[FAILURE]\r\n");
	}

	System_flush();
}

void test_is_All_Encoder_Are_Matched_With_Wheels(){
	bool lSuccess = true;
	System_printf("*******Encoder testing********\r\n");
	lSuccess &= test_is_Encoder_is_Matched_With_Wheel(0);;
	lSuccess &= test_is_Encoder_is_Matched_With_Wheel(1);
	lSuccess &= test_is_Encoder_is_Matched_With_Wheel(2);
	lSuccess &= test_is_Encoder_is_Matched_With_Wheel(3);

	if(lSuccess){
			System_printf("[SUCCESS]\r\n");
	}
	else{
		System_printf("[FAILURE]\r\n");
	}
	System_flush();

}

bool test_is_Encoder_is_Matched_With_Wheel(uint8_t pMotorId){

	//flush encodeur
	quad_readCounters(&HandleRobot.HandleQuad[0]);
	quad_readCounters(&HandleRobot.HandleQuad[1]);

	dcMotor_setPWM(&HandleRobot.HandleMotor[pMotorId],900);
	test_wait();
	dcMotor_setPWM(&HandleRobot.HandleMotor[pMotorId],EPWM_BRAKE);
	test_wait();

	quad_readCounters(&HandleRobot.HandleQuad[0]);
	quad_readCounters(&HandleRobot.HandleQuad[1]);

	int16_t lCount = 0;
	if(pMotorId == 0){
		lCount = HandleRobot.HandleQuad[0].Count0;
	}
	else if(pMotorId == 1){
		lCount = HandleRobot.HandleQuad[0].Count1;
	}
	else if(pMotorId == 2){
		lCount = HandleRobot.HandleQuad[1].Count0;
	}
	else if(pMotorId == 3){
		lCount = HandleRobot.HandleQuad[1].Count1;
	}

	if(lCount != 0){
		System_printf("Encoder %d [OK]\r\n",pMotorId);
		return true;
	}
	else{
		System_printf("Encoder %d [Fail]\r\n",pMotorId);
		return false;
	}
}

void test_is_Directions_Ok(){
	bool lSuccess = true;
	System_printf("*******Direction testing********\r\n");
	lSuccess &= test_is_Wheel_Direction_Left(0);
	System_flush();
	lSuccess &= test_is_Wheel_Direction_Left(1);
	System_flush();
	lSuccess &= test_is_Wheel_Direction_Left(2);
	System_flush();
	lSuccess &= test_is_Wheel_Direction_Left(3);
	System_flush();

	lSuccess &= test_is_Wheel_Direction_Right(0);
	System_flush();
	lSuccess &= test_is_Wheel_Direction_Right(1);
	System_flush();
	lSuccess &= test_is_Wheel_Direction_Right(2);
	System_flush();
	lSuccess &= test_is_Wheel_Direction_Right(3);
	System_flush();

	if(lSuccess){
		System_printf("[SUCCESS]\r\n");
	}
	else{
		System_printf("[FAILURE]\r\n");
	}
	System_flush();
}

bool test_is_Wheel_Direction_Left(uint8_t pMotorId){
	//flush encodeur
	quad_readCounters(&HandleRobot.HandleQuad[0]);
	quad_readCounters(&HandleRobot.HandleQuad[1]);

	dcMotor_setDirection(&HandleRobot.HandleMotor[pMotorId],LEFT);
	dcMotor_setPWM(&HandleRobot.HandleMotor[pMotorId],900);
	test_wait();
	dcMotor_setPWM(&HandleRobot.HandleMotor[pMotorId],EPWM_BRAKE);
	test_wait();

	quad_readCounters(&HandleRobot.HandleQuad[0]);
	quad_readCounters(&HandleRobot.HandleQuad[1]);

	int16_t lCount = 0;
	if(pMotorId == 0){
		lCount = HandleRobot.HandleQuad[0].Count0;
	}
	else if(pMotorId == 1){
		lCount = HandleRobot.HandleQuad[0].Count1;
	}
	else if(pMotorId == 2){
		lCount = HandleRobot.HandleQuad[1].Count0;
	}
	else if(pMotorId == 3){
		lCount = HandleRobot.HandleQuad[1].Count1;
	}

	if(lCount < 0){
		System_printf("Left Direction Motor %d [OK]\r\n",pMotorId);
		return true;
	}
	else{
		System_printf("Left Direction Motor %d [Fail]\r\n",pMotorId);
		return false;
	}
}

bool test_is_Wheel_Direction_Right(uint8_t pMotorId){
	//flush encodeur
	quad_readCounters(&HandleRobot.HandleQuad[0]);
	quad_readCounters(&HandleRobot.HandleQuad[1]);

	dcMotor_setDirection(&HandleRobot.HandleMotor[pMotorId],RIGHT);
	dcMotor_setPWM(&HandleRobot.HandleMotor[pMotorId],900);
	test_wait();
	dcMotor_setPWM(&HandleRobot.HandleMotor[pMotorId],EPWM_BRAKE);
	test_wait();

	quad_readCounters(&HandleRobot.HandleQuad[0]);
	quad_readCounters(&HandleRobot.HandleQuad[1]);

	int16_t lCount = 0;
	if(pMotorId == 0){
		lCount = HandleRobot.HandleQuad[0].Count0;
	}
	else if(pMotorId == 1){
		lCount = HandleRobot.HandleQuad[0].Count1;
	}
	else if(pMotorId == 2){
		lCount = HandleRobot.HandleQuad[1].Count0;
	}
	else if(pMotorId == 3){
		lCount = HandleRobot.HandleQuad[1].Count1;
	}

	if(lCount > 0){
		System_printf("Right Direction Motor %d [OK]\r\n",pMotorId);
		return true;
	}
	else{
		System_printf("Right Direction Motor %d [Fail]\r\n",pMotorId);
		return false;
	}
}

void test_wait(){
	uint32_t i,j = 0;
	for(i = 0; i < 10000;i++){
		for(j = 0; j < 10;j++);
	}
}

