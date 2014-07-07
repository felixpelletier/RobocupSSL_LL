/*
 * two_wheel_ctrl.c
 *
 *  Created on: 2014-01-06
 *      Author: Philippe Babin
 */

#include "four_wheel_ctrl.h"

void fourWheelCtrl_Init(){

}
// v1 = Tsin(te - 45) + 0.18PI W
// v2 = Tsin(te - 135) + 0.18PI W
// v3 = Tsin(te - 225) + 0.18PI W
// v4 = Tsin(te - 315) + 0.18PI W

void fourWheelCtrl_Update( _iq pX, _iq pY, _iq pTheta){

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
	_iq v3 = _IQmpy(magnitude, _IQsin(angle + _IQ(5.4977825)));

	HandleRobot.HandlePid[0].term.Ref = v2 + pTheta;
	HandleRobot.HandlePid[1].term.Ref = v3 + pTheta;
	HandleRobot.HandlePid[2].term.Ref = v0 + pTheta;
	HandleRobot.HandlePid[3].term.Ref = v1 + pTheta;

	if(_IQtoF(HandleRobot.HandlePid[0].term.Ref) < 0 ){dcMotor_setDirection(&HandleRobot.HandleMotor[0],RIGHT);}
	else{dcMotor_setDirection(&HandleRobot.HandleMotor[0],LEFT);}

	if(_IQtoF(HandleRobot.HandlePid[1].term.Ref) < 0 ){dcMotor_setDirection(&HandleRobot.HandleMotor[1],RIGHT);}
	else{dcMotor_setDirection(&HandleRobot.HandleMotor[1],LEFT);}

	if(_IQtoF(HandleRobot.HandlePid[2].term.Ref) < 0 ){dcMotor_setDirection(&HandleRobot.HandleMotor[2],RIGHT);}
	else{dcMotor_setDirection(&HandleRobot.HandleMotor[2],LEFT);}

	if(_IQtoF(HandleRobot.HandlePid[3].term.Ref) < 0 ){dcMotor_setDirection(&HandleRobot.HandleMotor[3],RIGHT);}
	else{dcMotor_setDirection(&HandleRobot.HandleMotor[3],LEFT);}

	HandleRobot.HandlePid[0].term.Ref=_IQabs(HandleRobot.HandlePid[0].term.Ref);
	HandleRobot.HandlePid[1].term.Ref=_IQabs(HandleRobot.HandlePid[1].term.Ref);
	HandleRobot.HandlePid[2].term.Ref=_IQabs(HandleRobot.HandlePid[2].term.Ref);
	HandleRobot.HandlePid[3].term.Ref=_IQabs(HandleRobot.HandlePid[3].term.Ref);
	//System_printf("%f %f %f %f\r\n", _IQtoF(HandleRobot.HandlePid[0].term.Ref), _IQtoF(HandleRobot.HandlePid[1].term.Ref), _IQtoF(HandleRobot.HandlePid[2].term.Ref), _IQtoF(HandleRobot.HandlePid[3].term.Ref));

	//System_printf("%f %f %f %f\r\n", _IQtoF(HandleRobot.HandlePid[0].term.Ref), _IQtoF(HandleRobot.HandlePid[1].term.Ref), _IQtoF(HandleRobot.HandlePid[2].term.Ref), _IQtoF(HandleRobot.HandlePid[3].term.Ref));
	//HandleRobot.HandlePid[1].term.Ref = v + diff;
	//HandleRobot.HandlePid[1].ref = v + diff;
}
