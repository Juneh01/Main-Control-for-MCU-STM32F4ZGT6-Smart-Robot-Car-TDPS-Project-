/*
 * motorset.h
 *
 *  Created on: Apr 9, 2024
 *      Author: zeng
 */

#ifndef MOTOR_MOTORSET_H_
#define MOTOR_MOTORSET_H_

#include "main.h"
#include "tim.h"

#define AIN1_RESET  HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET)//设置AIN1 PB13为 低电平
#define AIN1_SET    HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET)//设置AIN1 PB13为 高电平

#define BIN1_RESET 	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET)  //设置BIN1 PB3为低电平
#define BIN1_SET    HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET)//设置AIN1 PB13为 高电平

void Motor_Set (int motor1, int motor2);

#endif /* MOTOR_MOTORSET_H_ */
