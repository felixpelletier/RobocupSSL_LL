/* =================================================================================
File name: pid.h
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

#ifndef PID_H_
#define PID_H_

#include <stdio.h>
#include "Robocup_Define.h"


PID_Handle pid_init(_iq pKp, _iq pKi, _iq pKd, _iq pUmax, _iq pUmin);
_iq pid_update(PID_Handle *pPid, _iq pFbk);
void pid_set(PID_Handle *pPid,_iq pKp, _iq pKi, _iq pKd);
void pid_display(PID_Handle *pPid);

#endif /* PID_H_ */
