/*
 * motor.c
 *
 *  Created on: Apr 4, 2023
 *      Author: 77454
 */
#include "motor.h"
#include "../../Peripherals/pid/pid.h"
/**
 *    @brief 控制电机进行正转、反转、停止
 *    @param None
 *    @retval None
 */
#define MAX_SPEED_UP  2
extern float Motor1Speed ;
extern float Motor2Speed ;
extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;
float motorSpeedUpCut = 0.5;//加减速速度变量

void LeftMotor_Go() //左电机正转 AIN输出相反电平  BIN也输出相反电平
{
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
}
void LeftMotor_Back()  //左电机反转
{
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
}
void LeftMotor_Stop()  //左电机停止 AIN和BIN输出相同电平
{
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
}
void RightMotor_Go() //右电机正转 AIN输出相反电平  BIN也输出相反电平
{
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
}
void RightMotor_Back()  //右电机反转
{
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}
void RightMotor_Stop()  //右电机停止 AIN和BIN输出相同电平
{
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}

/**
 *    @brief 控制电机进行速度控制
 *    @param 运动方向，左右电机的PWM值
 *    @retval None
 */
void MotorControl(char motorDirection, int leftMotorPWM, int rightMotorPWM) {
	switch (motorDirection) {
	case 0:
		LeftMotor_Go();
		RightMotor_Go();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, leftMotorPWM);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, rightMotorPWM);
		break;
	case 1:
		LeftMotor_Back();
		RightMotor_Back();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, leftMotorPWM);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, rightMotorPWM);
		break;
	case 2:
		LeftMotor_Stop();
		RightMotor_Stop();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		break;
	default:
		break;
	}
}

void motorPidSetSpeed(float Motor1SetSpeed,float Motor2SetSpeed)
{
//改变电机PID参数的目标速度
	pidMotor1Speed.target_val = Motor1SetSpeed;
	pidMotor2Speed.target_val = Motor2SetSpeed;
//根据PID计算 输出作用于电机
	MotorControl(0, PID_realize(&pidMotor1Speed, Motor1Speed), PID_realize(&pidMotor2Speed, Motor2Speed));
}

//向前加速函数
void motorSpeedUp(void)
{
	static float MotorSetSpeedUp=0.5;//静态变量 函数结束 变量不会销毁
	if(MotorSetSpeedUp <= MAX_SPEED_UP) MotorSetSpeedUp +=0.5 ; //如果没有超过最大值就增加0.5
	motorPidSetSpeed(MotorSetSpeedUp,MotorSetSpeedUp);//设置到电机
}
//向前减速函数
void motorSpeedCut(void)
{
	static float MotorSetSpeedCut=3;//静态变量 函数结束 变量不会销毁
	if(MotorSetSpeedCut >=0.5) MotorSetSpeedCut-=0.5;//判断是否速度太小
	motorPidSetSpeed(MotorSetSpeedCut,MotorSetSpeedCut);//设置到电机
}
