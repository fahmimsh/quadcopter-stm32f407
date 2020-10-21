/*
 * pid.h
 *
 *  Created on: Aug 18, 2020
 *      Author: LENOVO
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"
#include <math.h>

//void PIDControlYAW(PIDType_t *pidtype);
//void PIDControlROLL(PIDType_t *pidtype);
//void PIDControlPITCH(PIDType_t *pidtype);
void PIDReset(PIDType_t *pidtype);
void PIDInit(PIDType_t *pidtype, double kp, double ki, double kd, double timesampling);
void PIDControl(PIDType_t *pidtype, float input, float setPoint);
void PIDControlAltitude(PIDType_t *pidtype, float input, float setPoint);
void trustControl(FLY_MODE fly_mode);

#endif /* INC_PID_H_ */
