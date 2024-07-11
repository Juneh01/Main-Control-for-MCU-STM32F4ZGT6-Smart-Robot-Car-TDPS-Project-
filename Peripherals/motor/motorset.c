/*
 * motorset.c
 *
 *  Created on: Apr 9, 2024
 *      Author: zeng
 */

/*******************
*  @brief  设置两个电机转速和方向
*  @param  motor1:电机B设置参数、motor2:设置参数
*  @param  motor1: 输入1~100 对应控制B电机正方向速度在1%-100%、输入-1~-100 对应控制B电机反方向速度在1%-100%、motor2同理
*  @return  无
*
*******************/
#include "motorset.h"

void Motor_Set (int motor1,int motor2)
{
	//根据参数正负 设置选择方向
	if(motor1 < 0) BIN1_SET;
	   else      BIN1_RESET;
	if(motor2 < 0) AIN1_SET;
		else      AIN1_RESET;

	//motor1 设置电机B的转速
	if(motor1 < 0)
	{
		if(motor1 < -99) motor1 = -99;//超过PWM幅值
		//负的时候绝对值越小  PWM占空比越大
		//现在的motor1      -1   -99
		//给寄存器或者函数  99  1
		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (100+motor1));//修改定时器1 通道1 PA8 Pulse改变占空比
	}
	else{
		if(motor1 > 99) motor1 = 99;
		//现在是   0 1  99
		//我们赋值 0 1 99
		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor1);//修改定时器1 通道1 PA8 Pulse改变占空比
	}

	//motor2 设置电机A的转速
	if(motor2 < 0)
	{
		if(motor2 < -99) motor2 = -99;//超过PWM幅值
		//负的时候绝对值越小  PWM占空比越大
		//现在的motor2      -1   -99
		//给寄存器或者函数   99  1
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (100+motor2));//修改定时器1 通道4 PA11 Pulse改变占空比
	}
	else{
		if(motor2 > 99) motor2 = 99;
		//现在是   0 1 99
		//我们赋值 0 1 99
		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, motor2);//修改定时器1 通道4 PA11 Pulse改变占空比

	}

}
