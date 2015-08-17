/* =================================================================================
File name: pid.c
Originator: C2000 System Applications, Texas Instruments
Description: Data and macro definitions for PID controller
=====================================================================================
History:
-------------------------------------------------------------------------------------
06-07-2010 Version 1.0
-------------------------------------------------------------------------------------
19-12-2013 Version 1.1
-------------------------------------------------------------------------------------
*/
#include "pid.h"

PID_Handle pid_init(_iq pKp, _iq pKi, _iq pKd, _iq pUmax, _iq pUmin){
	PID_Handle lPid;

	lPid.param.Kr = _IQ(1);   //pid Constant
	lPid.param.Kp = pKp;
	lPid.param.Ki = pKi;
	lPid.param.Kd = pKd;
	lPid.param.Km = _IQ(1);

	lPid.param.Umax = pUmax;  //saturation filter
	lPid.param.Umin = pUmin;

	lPid.term.c1 = _IQ(1);  //D filter
	lPid.term.c2 = _IQ(0);
	lPid.data.d1 = _IQ(0);
	lPid.data.d2 = _IQ(0);
	lPid.data.i1 = _IQ(0);
	lPid.data.ud = _IQ(0);
	lPid.data.ui = _IQ(0);
	lPid.data.up = _IQ(0);
	lPid.data.v1 = _IQ(0);
	lPid.data.w1 = _IQ(0); // I weight, when no saturation occur, the w1 = 1.0

	return lPid;
}

_iq pid_update(PID_Handle *pPid, _iq pFbk){
	pPid->term.Fbk = pFbk;
	//v->term.Ref = Ref;


	/* proportional term */
	// Kr * Ref - Fbk
	pPid->data.up = _IQmpy(pPid->param.Kr, pPid->term.Ref) - pPid->term.Fbk;
	/* integral term */
	// Ki * (w1 * (Ref - Fbk)) + i1
	// w1 = 1 when no saturation occur
	pPid->data.ui = _IQmpy(pPid->param.Ki, _IQmpy(pPid->data.w1, (pPid->term.Ref - pPid->term.Fbk))) + pPid->data.i1;
	pPid->data.i1 = pPid->data.ui;

	/* derivative term */
	// Kd * (c1 * (Ref * Km  - Fbk) ) - d2
	pPid->data.d2 = _IQmpy(pPid->param.Kd,
						_IQmpy(pPid->term.c1, (_IQmpy(pPid->term.Ref, pPid->param.Km), - pPid->term.Fbk))) - pPid->data.d2;
	pPid->data.ud = pPid->data.d2 + pPid->data.d1;
	pPid->data.d1 = _IQmpy(pPid->data.ud, pPid->term.c1);

	/* control output */
	// v1 = Kp * (up + ui + ud)
	pPid->data.v1 = _IQmpy(pPid->param.Kp, pPid->data.up) + pPid->data.ui + pPid->data.ud; // previously kp ( up + ui + ud)
	// out = Kp * (up + ui + ud)
	pPid->term.Out = _IQsat(pPid->data.v1, pPid->param.Umax, pPid->param.Umin);
	//pPid->data.w1 = (pPid->term.Out == pPid->data.v1) ? _IQ(1.0) : _IQ(1.0);

	return pPid->term.Out;
}

void pid_set(PID_Handle *pPid,_iq pKp, _iq pKi, _iq pKd){
	pPid->param.Kp = pKp;
	pPid->param.Ki = pKi;
	pPid->param.Kd = pKd;
}

void pid_display(PID_Handle *pPid){
	System_printf("pid =%f error=%f\n\r",_IQtoF(pPid->term.Out), _IQtoF(pPid->term.Ref - pPid->term.Fbk));
}
