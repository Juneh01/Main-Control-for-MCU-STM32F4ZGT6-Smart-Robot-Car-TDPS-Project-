/*
 * motor.h
 *
 *  Created on: Apr 4, 2023
 *      Author: 77454
 */

#ifndef MOTOR_MOTOR_H_
#define MOTOR_MOTOR_H_

#include "main.h"
#include "tim.h"


void MotorControl(char motorDirection,int leftMotorPWM, int rightMotorPWM);
void motorPidSetSpeed(float Motor1SetSpeed,float Motor2SetSpeed);
void motorSpeedUp(void);
void motorSpeedCut(void);

#endif /* MOTOR_MOTOR_H_ */

