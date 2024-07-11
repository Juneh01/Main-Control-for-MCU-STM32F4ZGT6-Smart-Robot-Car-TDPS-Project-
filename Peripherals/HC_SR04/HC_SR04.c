/*
 * HC_SR04.c
 *
 *  Created on: Apr 29, 2024
 *      Author: admin
 */
#include "HC_SR04.h"
#include "main.h"

void HC_SR04_Delayus(uint32_t usdelay)
{
  __IO uint32_t Delay = usdelay * (SystemCoreClock / 8U / 1000U/1000);//SystemCoreClock:系统频率
  do
  {
    __NOP();
  }
  while (Delay --);
}

float HC_SR04_Read1(void)
{
	uint32_t i = 0;
	float Distance;
	HAL_GPIO_WritePin(HC_SR04_Trig1_GPIO_Port,HC_SR04_Trig1_Pin,GPIO_PIN_SET);//输出15us高电平
	HC_SR04_Delayus(15);
	HAL_GPIO_WritePin(HC_SR04_Trig1_GPIO_Port,HC_SR04_Trig1_Pin,GPIO_PIN_RESET);//高电平输出结束，设置为低电平

	while(HAL_GPIO_ReadPin(HC_SR04_Echo1_GPIO_Port,HC_SR04_Echo1_Pin) == GPIO_PIN_RESET)//等待回响高电平
	{
		i++;
		HC_SR04_Delayus(1);
		if(i>100000) return -1;//超时退出循环、防止程序卡死这里
	}
	i = 0;
	while(HAL_GPIO_ReadPin(HC_SR04_Echo1_GPIO_Port,HC_SR04_Echo1_Pin) == GPIO_PIN_SET)//下面的循环是2us
	{
		i = i+1;
		HC_SR04_Delayus(1);//1us 延时，但是整个循环大概2us左右
		if(i >100000) return -2;//超时退出循环
	}
	Distance = i*2*0.033/2;//这里乘2的原因是上面是2微妙
	return Distance	;
}

float HC_SR04_Read2(void)
{
	uint32_t j = 0;
	float Distance2;
	HAL_GPIO_WritePin(HC_SR04_Trig2_GPIO_Port,HC_SR04_Trig2_Pin,GPIO_PIN_SET);//输出15us高电平
	HC_SR04_Delayus(15);
	HAL_GPIO_WritePin(HC_SR04_Trig2_GPIO_Port,HC_SR04_Trig2_Pin,GPIO_PIN_RESET);//高电平输出结束，设置为低电平

	while(HAL_GPIO_ReadPin(HC_SR04_Echo2_GPIO_Port,HC_SR04_Echo2_Pin) == GPIO_PIN_RESET)//等待回响高电平
	{
		j++;
		HC_SR04_Delayus(1);
		if(j>100000) return -1;//超时退出循环、防止程序卡死这里
	}
	j = 0;
	while(HAL_GPIO_ReadPin(HC_SR04_Echo2_GPIO_Port,HC_SR04_Echo2_Pin) == GPIO_PIN_SET)//下面的循环是2us
	{
		j = j+1;
		HC_SR04_Delayus(1);//1us 延时，但是整个循环大概2us左右
		if(j >100000) return -2;//超时退出循环
	}
	Distance2 = j*2*0.033/2;//这里乘2的原因是上面是2微妙
	return Distance2	;
}
