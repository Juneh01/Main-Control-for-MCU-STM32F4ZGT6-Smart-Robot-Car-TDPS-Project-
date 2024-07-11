/*
 * openmv.c
 *
 *  Created on: Mar 12, 2024
 *      Author: zeng
 */
#include "openmv.h"
//#include "stdio.h"
#include "usart.h"
#include "string.h"
#include "stdint.h"
//#include "globepara.h"
//#include "main.h"

//uint8_t u_buf[64];
//
//#define printf(...)  HAL_UART_Transmit_DMA((UART_HandleTypeDef * )&huart1, (uint8_t *)u_buf,\
//											sprintf((char *)u_buf,__VA_ARGS__));

//static uint8_t  Bias=0,Bias_Sign=0,label_value=0,Ch=0;
//extern uint8_t  Bias, Bias_Sign, label_value ,Ch;
signed int  Bias;
signed int  Bias_Sign;
signed int  label_value;
signed int  Ch;


void   Openmv_Receive_Data(int16_t com_data)
{

    uint8_t i;
		static uint8_t RxCounter1=0;//计数
		static uint16_t RxBuffer1[10]={0};
		static uint8_t RxState = 0;
		static uint8_t RxFlag1 = 0;

		if(RxState==0&&com_data==0x2C)  //0x2c帧头
				{

					RxState=1;
					RxBuffer1[RxCounter1++]=com_data;
					HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
				}

		else if(RxState==1&&com_data==0x12)  //0x12帧头
				{
					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
					RxState=2;
					RxBuffer1[RxCounter1++]=com_data;
				}
		else if(RxState==2)
				{

					RxBuffer1[RxCounter1++]=com_data;
		if(RxCounter1>=10||com_data == 0x5B)       //RxBuffer1接受满了,接收数据结束
					{
						RxState=3;
						RxFlag1=1;

						Bias=RxBuffer1[RxCounter1-5];
						Bias_Sign=RxBuffer1[RxCounter1-4];
						label_value=RxBuffer1[RxCounter1-3];
						Ch=RxBuffer1[RxCounter1-2];
						//printf("%d %d %d %d \r\n",Bias,Bias_Sign,label_value,Ch);
					}
			}

				else if(RxState==3)		//检测是否接受到结束标志
				{
						if(RxBuffer1[RxCounter1-1] == 0x5B)
						{

									RxFlag1 = 0;
									RxCounter1 = 0;
									RxState = 0;

						}
						else   //接收错误
						{
									RxState = 0;
									RxCounter1=0;
									for(i=0;i<10;i++)
									{
											RxBuffer1[i]=0x00;      //将存放数据数组清零
									}
						}
				}

				else   //接收异常
				{
						RxState = 0;
						RxCounter1=0;
						for(i=0;i<10;i++)
						{
								RxBuffer1[i]=0x00;      //将存放数据数组清零
						}
				}
      }





