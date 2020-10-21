/*
 * pid.c
 *
 *  Created on: Aug 18, 2020
 *      Author: LENOVO
 */

#include "pid.h"
extern TIM_HandleTypeDef htim2;

void PIDControl(PIDType_t *pidtype, float dataSensor, float setPoint){
	pidtype->setPoint = constrain(setPoint, -30, 30);
	pidtype->error = pidtype->setPoint - dataSensor;

	if(pidtype->error >= 180) pidtype->error -= 360;
	else if(pidtype->error < -180) pidtype->error += 360;

	pidtype->sumIntegral += pidtype->error * pidtype->timesampling;
	pidtype->sumIntegral = constrain(pidtype->sumIntegral, -500, 500);

	pidtype->derivative = (pidtype->error - pidtype->preverror) / pidtype->timesampling;
	pidtype->preverror = pidtype->error;

	pidtype->output = (pidtype->kp * pidtype->error) + (pidtype->kd * pidtype->derivative) + (pidtype->ki * pidtype->sumIntegral);
}
void PIDControlAltitude(PIDType_t *pidtype, float dataSensor, float setPoint){
	pidtype->setPoint = constrain(setPoint, 0, 100);
	pidtype->error = pidtype->setPoint - dataSensor;

	//if(pidtype->error >= 100) pidtype->error = 100;
	//else if(pidtype->error < -100) pidtype->error = -100;

	pidtype->sumIntegral += pidtype->error * pidtype->timesampling;
	pidtype->sumIntegral = constrain(pidtype->sumIntegral, -1000, 1000);

	pidtype->derivative = (pidtype->error - pidtype->preverror) / pidtype->timesampling;
	pidtype->preverror = pidtype->error;

	pidtype->output = (pidtype->kp * pidtype->error) + (pidtype->kd * pidtype->derivative) + (pidtype->ki * pidtype->sumIntegral);
}

void PIDReset(PIDType_t *pidtype){
	pidtype->sumIntegral = 0;
	pidtype->output = 0;

	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1000);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,1000);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,1000);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,1000);
}
void PIDInit(PIDType_t *pidtype, double kp, double ki, double kd, double timesampling){
	PIDReset(pidtype);

	pidtype->kp = kp;
	pidtype->kd = kd;
	pidtype->ki = ki;

	pidtype->timesampling = timesampling;
}
void trustControl(FLY_MODE fly_mode){
	/*
	  		CW 2\     /1 CCW
				 \   /
				  \ /
				  / \
	           	 /   \
		   CCW 3/     \4 CW
		       QuadCopter
			   Motor 1000 KV
			   Propeller : 9 x 4.5
			   MaX thrust = 49.663985 N
			   Thrust for each motor = 12.41599
		Formula :
		F = 1.225 * ((3.14(0.0254 * d)^2)/4) *(RPM*0.0254*pitch*1/60)^2 * (d/(3.29546*pitch))^1.5;
		F = 0.050252560785 * 522.5796 * 0.47279287884410658042558653798071
	*/

	float motor1Thrust,motor2Thrust,motor3Thrust,motor4Thrust;
	float motor1Torque,motor2Torque,motor3Torque,motor4Torque;
	float thrust, Altitudetrush;
	int RPMmotor1,RPMmotor2,RPMmotor3,RPMmotor4;

	const float RADS = 57.29577795;
	const float angleMotor1 = 45;
	const float angleMotor2 = 135;
	const float angleMotor3 = 225;
	const float angleMotor4 = 315;
	const float L = 0.225;
	if(fly_mode == FLY_MODE_ON){
		thrust = map(inputThrottle, 1000, 2000, 0, 49.663985);
	} else if(fly_mode == FLY_MODE_HOLD){
		if (inputThrottle >= 1200){
			Altitudetrush = 1.3 * PIDAltitude.output;
			thrust = 12.74 + Altitudetrush;
		}else thrust = map(inputThrottle, 1000, 2000, 0, 49.663985);
		/*thrust = map(holdThrottle + PIDAltitude.output, 1000, 2000, 0, 49.663985);*/
	}


	motor1Torque = (thrust/4 - PIDPitch.output * sin(angleMotor1/RADS) + PIDRoll.output * cos(angleMotor1 / RADS) + PIDYaw.output) * L;
	motor2Torque = (thrust/4 - PIDPitch.output * sin(angleMotor2/RADS) + PIDRoll.output * cos(angleMotor2 / RADS) - PIDYaw.output) * L;
	motor3Torque = (thrust/4 - PIDPitch.output * sin(angleMotor3/RADS) + PIDRoll.output * cos(angleMotor3 / RADS) + PIDYaw.output) * L;
	motor4Torque = (thrust/4 - PIDPitch.output * sin(angleMotor4/RADS) + PIDRoll.output * cos(angleMotor4 / RADS) - PIDYaw.output) * L;

	motor1Thrust = motor1Torque/L;
	motor2Thrust = motor2Torque/L;
	motor3Thrust = motor3Torque/L;
	motor4Thrust = motor4Torque/L;

	RPMmotor1 = sqrt(motor1Thrust / 0.023759052) / 0.001905;
	RPMmotor2 = sqrt(motor2Thrust / 0.023759052) / 0.001905;
	RPMmotor3 = sqrt(motor3Thrust / 0.023759052) / 0.001905;
	RPMmotor4 = sqrt(motor4Thrust / 0.023759052) / 0.001905;

	RPMmotor1 = constrain(RPMmotor1,0,12000);
	RPMmotor2 = constrain(RPMmotor2,0,12000);
	RPMmotor3 = constrain(RPMmotor3,0,12000);
	RPMmotor4 = constrain(RPMmotor4,0,12000);

	pulseESC1 = map(RPMmotor1,0,12000,1000,2000);
	pulseESC2 = map(RPMmotor2,0,12000,1000,2000);
	pulseESC3 = map(RPMmotor3,0,12000,1000,2000);
	pulseESC4 = map(RPMmotor4,0,12000,1000,2000);

	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,pulseESC1);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,pulseESC2);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,pulseESC3);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,pulseESC4);

}
