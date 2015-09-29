/*
 * two_wheel_ctrl.c
 *
 *  Created on: 2014-01-06
 *      Author: Philippe Babin
 */

#include "four_wheel_ctrl.h"

void fourWheelCtrl_Init(){

}

/*    For robot ALPHA:
 *
 *         90deg
 *    m0 /      \  m1
 *      / o     o\				  y
 *         o   o                  ^
 *          o o                   |
 * 180deg    x          0deg      ---> x
 *          o o
 *         o   o
 *       \o     o /
 *     m3 \      / m2
 *     	  270 deg
 * m0 = v2 = -0.707 * (fX+fY);
 * m1 = v3 =  0.707 * (fX-fY);
 * m2 = v0 =  0.707 * (fX+fY);
 * m3 = v1 = -0.707 * (fX-fY);
 *
 * m0-3 => Name of the motor on the hardware and in the code
 * v0-3 => Stupid variable name to remove
 * On the BETA the m1 and m3 are interchange and the robot direction is inversed
 *
 * positive rotation are anti-clockwise
 */

void fourWheelCtrl_Update( float fX, float fY, float fTheta){
	//Conversion to float because iq doesn't work when pass has argument
	_iq pTheta = _IQ(fTheta);
	/*
	_iq magnitude = _IQmag(pX,pY);
	_iq angle = _IQatan2( pY, pX);

	// j = 0.18 * PI * w
	//_iq j = _IQmpy(w, _IQ(0.5654));
	// dephasage = 45 degrees = 0.785 radians
	_iq v0 = _IQmpy(magnitude, _IQsin(angle + _IQ(0.785398163)));
	// dephasage = 135 degrees = 2.3561925 radians
	_iq v1 = _IQmpy(magnitude, _IQsin(angle + _IQ(2.3561925)));
	// dephasage = 225 degrees = 3.9269875 radians
	_iq v2 = _IQmpy(magnitude, _IQsin(angle + _IQ(3.9269875)));
	// dephasage = 315 degrees = 5.4977825 radians
	_iq v3 = _IQmpy(magnitude, _IQsin(angle + _IQ(5.4977825)));*/

	float sqrt_two = 0.707;
	// j = 0.18 * PI * w
	//_iq j = _IQmpy(w, _IQ(0.5654));
	// dephasage = 45 degrees = 0.785 radians
	float vf1 = -sqrt_two * (fX-fY); // changement divisé par deux ancienne valeur 0.707
	float vf2 = -sqrt_two * (fX+fY);
	float vf3 =  sqrt_two * (fX-fY);
	float vf0 =  sqrt_two * (fX+fY);

#ifdef BETA
	float rRobot = pTheta * 0.082;

	HandleRobot.HandlePid[0].term.Ref = _IQ17(vf2) + _IQ17(rRobot);
	HandleRobot.HandlePid[1].term.Ref = _IQ17(vf1) + _IQ17(rRobot);
	HandleRobot.HandlePid[2].term.Ref = _IQ17(vf0) + _IQ17(rRobot);
	HandleRobot.HandlePid[3].term.Ref = _IQ17(vf3) + _IQ17(rRobot);
	DCMotor_DIR dir_right_alpha = LEFT;
	DCMotor_DIR dir_left_alpha = RIGHT;
#else // Alpha
	float rRobot = pTheta * 0.082;

	HandleRobot.HandlePid[0].term.Ref = _IQ17(vf2) + _IQ17(pTheta);
	HandleRobot.HandlePid[1].term.Ref = _IQ17(vf3) + _IQ17(pTheta);
	HandleRobot.HandlePid[2].term.Ref = _IQ17(vf0) + _IQ17(pTheta);
	HandleRobot.HandlePid[3].term.Ref = _IQ17(vf1) + _IQ17(pTheta);
	DCMotor_DIR dir_right_alpha = RIGHT;
	DCMotor_DIR dir_left_alpha = LEFT;
#endif // BETA


	if(_IQtoF(HandleRobot.HandlePid[0].term.Ref) > 0 ){dcMotor_setDirection(&HandleRobot.HandleMotor[0],dir_left_alpha);}
	else{dcMotor_setDirection(&HandleRobot.HandleMotor[0],dir_right_alpha);}

	if(_IQtoF(HandleRobot.HandlePid[1].term.Ref) > 0 ){dcMotor_setDirection(&HandleRobot.HandleMotor[1],dir_right_alpha);}
	else{dcMotor_setDirection(&HandleRobot.HandleMotor[1],dir_left_alpha);}

	if(_IQtoF(HandleRobot.HandlePid[2].term.Ref) > 0 ){dcMotor_setDirection(&HandleRobot.HandleMotor[2],dir_left_alpha);}
	else{dcMotor_setDirection(&HandleRobot.HandleMotor[2],dir_right_alpha);}

	if(_IQtoF(HandleRobot.HandlePid[3].term.Ref) > 0 ){dcMotor_setDirection(&HandleRobot.HandleMotor[3],dir_right_alpha);}
	else{dcMotor_setDirection(&HandleRobot.HandleMotor[3],dir_left_alpha);}


	HandleRobot.HandlePid[0].term.Ref=_IQabs(HandleRobot.HandlePid[0].term.Ref);
	HandleRobot.HandlePid[1].term.Ref=_IQabs(HandleRobot.HandlePid[1].term.Ref);
	HandleRobot.HandlePid[2].term.Ref=_IQabs(HandleRobot.HandlePid[2].term.Ref);
	HandleRobot.HandlePid[3].term.Ref=_IQabs(HandleRobot.HandlePid[3].term.Ref);

	/*
	System_printf("fX:%f fy:%f\n\r", fX, fY);
	System_printf("m0:%f m1:%f m2:%f m3:%f\n\r", vf2,
												 vf1,
												 vf0,
												 vf3);*/
	/*System_printf("px%f p+s%f v0%f absref%f\n\r",   fX,
													fX+fY,
													vf2,
													_IQtoF(HandleRobot.HandlePid[0].term.Ref));*/
	//System_printf("%f %f %f %f\r\n", _IQtoF(HandleRobot.HandlePid[0].term.Ref), _IQtoF(HandleRobot.HandlePid[1].term.Ref), _IQtoF(HandleRobot.HandlePid[2].term.Ref), _IQtoF(HandleRobot.HandlePid[3].term.Ref));

	//System_printf("%f %f %f %f\r\n", _IQtoF(HandleRobot.HandlePid[0].term.Ref), _IQtoF(HandleRobot.HandlePid[1].term.Ref), _IQtoF(HandleRobot.HandlePid[2].term.Ref), _IQtoF(HandleRobot.HandlePid[3].term.Ref));
	//HandleRobot.HandlePid[1].term.Ref = v + diff;
	//HandleRobot.HandlePid[1].ref = v + diff;
}
