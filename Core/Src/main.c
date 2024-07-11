/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdint.h"
#include "../../Peripherals/motor/motor.h"
#include "../../Peripherals/motor/motorset.h"
#include "../../Peripherals/openmv/openmv.h"
#include "../../Peripherals/MPU6050/mpu6050.h"
#include "../../Peripherals/MPU6050/eMPL/inv_mpu.h"
#include "../../Peripherals/MPU6050/eMPL/inv_mpu_dmp_motion_driver.h"
#include "../../Peripherals/niming/niming.h"
#include "../../Peripherals/pid/pid.h"
#include "../../Peripherals/CJSON/cJSON.h"
#include "../../Peripherals/HC_SR04/HC_SR04.h"

//#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* -------------------------------------------------------------------------------------------- Printf Redefine + */
uint8_t u_buf[64];

#define printf(...)  HAL_UART_Transmit_DMA((UART_HandleTypeDef * )&huart1, (uint8_t *)u_buf,\
											sprintf((char *)u_buf,__VA_ARGS__));
//#define printf(...)  HAL_UART_Transmit (&huart1, (uint8_t *)u_buf, sprintf((char *)u_buf,__VA_ARGS__), 100);
/* -------------------------------------------------------------------------------------------- Printf Redefine - */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//short Encode1Count = 0;
//short Encode2Count = 0;


extern long Encode1count_distance;
extern long Encode2count_distance;

extern long timer;

float distance=0;
//float Motor1Speed = 0.00;
//float Motor2Speed = 0.00;
extern float Motor1Speed ;
extern float Motor2Speed ;
int Motor1PWM;
int Motor2PWM;

extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;
extern tPid pidOpenMV_Tracking;
extern uint8_t Usart1_ReadBuf[255];	//串口1 缓冲数组
extern tPid pidMPU6050YawMovement;
float p,i,d,a;

uint8_t Usart3String[35];
float g_fHC_SR04_Read;//超声波传感器读取障碍物数�????????????????
float length1;
float length2;

float pitch,roll,yaw; // 俯仰�??????????????? 横滚�??????????????? 航向�???????????????
float g_fMPU6050YawMovePidOut = 0.00f; //姿�?�PID运算输出
float g_fMPU6050YawMovePidOut1 = 0.00f; //第一个电机控制输�??????????????
float g_fMPU6050YawMovePidOut2 = 0.00f; //第一个电机控制输�??????????????
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* -------------------------------------------------------------------------------------------- Usart 1 Buffer + */
uint8_t usart1_rxbuff;
/* -------------------------------------------------------------------------------------------- Usart 1 Buffer - */


/* ------------------------------------------------------------------------------------- OpenMV Usart 2 Buffer + */
uint8_t usart2_rxbuff;
/* ------------------------------------------------------------------------------------- OpenMV Usart 2 Buffer - */


/* ------------------------------------------------------------------------------------ Encoder Parameter Init + */
uint8_t direction1 = 2 ;
int32_t counter1 = 0 ;
float rotations1 = 0;
float distance1 =0 ;
float counter_last1 = 0;
float speed_w1 = 0;
float speed_v1 = 0;

uint8_t direction2 = 2 ;
int32_t counter2 = 0 ;
float rotations2 = 0;
float distance2 =0 ;
float counter_last2 = 0;
float speed_w2 = 0;
float speed_v2 = 0;

float Speed_Bias;
float Speed_Bias_PID_Out;
float Speed_Bias_PID_Out1;
float Speed_Bias_PID_Out2;
float Last_Bias;


float Speed_length_bais;
float Speed_length_Bias_PID_Out;
float Speed_length_Bias_PID_Out1;
float Speed_length_Bias_PID_Out2;
float Last_length;



float yaw_Rightobstacle=0;
extern float Motor1Speed;
extern float Motor2Speed;
int ffflag=0;


int gate=0;
int label_value_tem=0;
int read_arrow=0;

int light_judge=0;

/* ------------------------------------------------------------------------------------ Encoder Parameter Init - */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	switch(GPIO_Pin)
//	{
//		case Rightobstacle_inter_Pin:
//			Rightobstacle();
//			break;
//		case KEY2_Pin:
//			LED2_TOGGLE();
//			break;
//		case KEY3_Pin:
//			BEEP_TOGGLE();
//			break;
//	}
//}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* --------------------------------------------------------------------------------- OpenMV PID + */
void Lane_Keeping(int Bias)
  {
	  //printf("bias:%d, sign:%d, color:%d, sign:%d \r\n",Bias,Bias_Sign,Cw,Ch);
	  if(Bias > 100){
		  Bias = 0;
	  }

	  if((Bias < 20 && Bias_Sign == 0) || (Bias < 20 && Bias_Sign == 1)) //直行
	  {
		  Speed_Bias = 0; //两边差�?? 左轮-右轮
	  }
	  else if(Bias < 30 && Bias > 20 && Bias_Sign == 0)//偏左 右转 极小
	  	  {
	  		  Speed_Bias = 0.05; //两边差�?? 左轮-右轮
	  	  }
	  	  else if(Bias < 30 && Bias > 20 && Bias_Sign == 1)//偏右 左转 极小
	  	  {
	  		  Speed_Bias = -0.09; //两边差�?? 左轮-右轮
	  	  }
	  else if(Bias < 50 && Bias > 30 && Bias_Sign == 0)//偏左 右转 �??????
	  {
		  Speed_Bias = 0.15; //两边差�?? 左轮-右轮
	  }
	  else if(Bias < 50 && Bias > 30 && Bias_Sign == 1)//偏右 左转 �??????
	  {
		  Speed_Bias = -0.17; //两边差�?? 左轮-右轮
	  }
	  else if(Bias > 50 && Bias_Sign == 0)//偏左 右转 �??????
	  {
		  Speed_Bias = 0.4; //两边差�?? 左轮-右轮
	  }
	  else if(Bias > 50 && Bias_Sign == 1)//偏右 左转 �??????
	  {
		  Speed_Bias = -0.45; //两边差�?? 左轮-右轮
	  }
	  else{
		  Speed_Bias = 0;
	  }
	  Speed_Bias_PID_Out = PID_realize(&pidOpenMV_Tracking, Speed_Bias);

	  Speed_Bias_PID_Out1 = 0.5 - Speed_Bias_PID_Out;
	  Speed_Bias_PID_Out2 = 0.5 + Speed_Bias_PID_Out;
	  if (Speed_Bias_PID_Out1 > 3) Speed_Bias_PID_Out1 = 3;
	  if (Speed_Bias_PID_Out1 < 0) Speed_Bias_PID_Out1 = 0;
	  if (Speed_Bias_PID_Out1 > 3) Speed_Bias_PID_Out1 = 3;
	  if (Speed_Bias_PID_Out1 < 0) Speed_Bias_PID_Out1 = 0;
	  if (Bias != Last_Bias)
	  {
		  motorPidSetSpeed(Speed_Bias_PID_Out1, Speed_Bias_PID_Out2);
	  }
	  Last_Bias = Bias;
  }
  /* --------------------------------------------------------------------------------- OpenMV PID - */





  /* --------------------------------------------------------------------------------- MPU PID - */
  void MPU_Keeping()
  {
	  mpu_dmp_get_data(&pitch,&roll,&yaw);//返回�???????????????:0,DMP成功解出欧拉�???????????????
	  while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} //这个可以解决经常读不出数据的�???????????????
	  g_fMPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);//PID计算输出目标速度 这个速度，会和基�??????????????速度加减
	  g_fMPU6050YawMovePidOut1 = 0.5 + g_fMPU6050YawMovePidOut;//基础速度加减PID输出速度
	  g_fMPU6050YawMovePidOut2 = 0.5 - g_fMPU6050YawMovePidOut;
	  if(g_fMPU6050YawMovePidOut1 >2) g_fMPU6050YawMovePidOut1 =1.5;//进行限幅
	  if(g_fMPU6050YawMovePidOut1 <0) g_fMPU6050YawMovePidOut1 =0;
	  if(g_fMPU6050YawMovePidOut2 >2) g_fMPU6050YawMovePidOut2 =1.5;
	  if(g_fMPU6050YawMovePidOut2 <0) g_fMPU6050YawMovePidOut2 =0;
	  motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);
	  HAL_Delay(30);

  }


  void MPU_Keeping_right()
  {
	  mpu_dmp_get_data(&pitch,&roll,&yaw);//返回�???????????????:0,DMP成功解出欧拉�???????????????
	  while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} //这个可以解决经常读不出数据的�???????????????
	  g_fMPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);//PID计算输出目标速度 这个速度，会和基�??????????????速度加减
	  g_fMPU6050YawMovePidOut1 = 0;//基础速度加减PID输出速度
	  g_fMPU6050YawMovePidOut2 = 0.2 - g_fMPU6050YawMovePidOut;
	  if(g_fMPU6050YawMovePidOut1 >0.3) g_fMPU6050YawMovePidOut1 =0.3;//进行限幅
	  if(g_fMPU6050YawMovePidOut1 <0) g_fMPU6050YawMovePidOut1 =0;
	  if(g_fMPU6050YawMovePidOut2 >0.3) g_fMPU6050YawMovePidOut2 =0.3;
	  if(g_fMPU6050YawMovePidOut2 <0) g_fMPU6050YawMovePidOut2 =0;
	  motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);
	  HAL_Delay(30);

  }


  void MPU_Keeping_left()
  {
	  mpu_dmp_get_data(&pitch,&roll,&yaw);//返回�???????????????:0,DMP成功解出欧拉�???????????????
	  while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} //这个可以解决经常读不出数据的�???????????????
	  g_fMPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);//PID计算输出目标速度 这个速度，会和基�??????????????速度加减
	  g_fMPU6050YawMovePidOut1 = 0.2 + g_fMPU6050YawMovePidOut;//基础速度加减PID输出速度
	  g_fMPU6050YawMovePidOut2 = 0;
	  if(g_fMPU6050YawMovePidOut1 >0.3) g_fMPU6050YawMovePidOut1 =0.3;//进行限幅
	  if(g_fMPU6050YawMovePidOut1 <0) g_fMPU6050YawMovePidOut1 =0;
	  if(g_fMPU6050YawMovePidOut2 >0.3) g_fMPU6050YawMovePidOut2 =0.3;
	  if(g_fMPU6050YawMovePidOut2 <0) g_fMPU6050YawMovePidOut2 =0;
	  motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);
	  HAL_Delay(30);

  }

/* --------------------------------------------------------------------------------- MPU PID - */

  /* --------------------------------------------------------------------------------- arrowchoice PID - */

//  void first_turn(void)
//  {
//	  motorPidSetSpeed(0,0);
//	  MPU_Init(); //初始化MPU6050
//	  while(MPU_Init()!=0)
//	  {
//		  printf("%d",MPU_Init());
//	  };
//	  while(mpu_dmp_init()!=0)
//	  {
//	  	  printf("%d",mpu_dmp_init());
//
//	    };
//
//	  HAL_Delay(5000);
//	  Encode1count_distance=0;
//	  Encode2count_distance=0;
//
//	  mpu_dmp_get_data(&pitch,&roll,&yaw);//返回�???????????????:0,DMP成功解出欧拉�???????????????
//	  while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} //这个可以解决经常读不出数据的�???????????????
//
//
//
//	  yaw_Rightobstacle=yaw;
//	  while(1)
//  	  {
//	  distance=(Encode1count_distance+Encode2count_distance)*3.14*6.5/(4*30*11);
//	  MPU_Keeping();
//	  printf("distance:%d\r\n",distance);//显示距离
//	  if(distance<280)
//	  {
//		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle;
//	  }
//	  else if(distance<650&&distance>280)
//	  {
//		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle+78.6;
//	  }
//	  else if(distance>650)
//	  {
//		  break;
//	  }
//  	  }
//	  HAL_Delay(2000);
//	  arrowchoice_left();
//  }



    void arrowchoice_left(void)
    {
  	  motorPidSetSpeed(0,0);
  	  MPU_Init(); //初始化MPU6050
  	  while(MPU_Init()!=0)
  	  {
  		  printf("%d",MPU_Init());
  	  };
  	  while(mpu_dmp_init()!=0)
  	  {
  	  	  printf("%d",mpu_dmp_init());

  	    };
  	  Encode1count_distance=0;
  	  Encode2count_distance=0;

  	  mpu_dmp_get_data(&pitch,&roll,&yaw);//返回�???????????????:0,DMP成功解出欧拉�???????????????
  	  while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} //这个可以解决经常读不出数据的�???????????????


  	  yaw_Rightobstacle=yaw;
  	  while(1)
    	  {
  	  distance=(Encode1count_distance+Encode2count_distance)*3.14*6.5/(4*30*11);
  	  MPU_Keeping();
  	  printf("distance:%d\r\n",distance);//显示距离
  	  if(distance<170)
  	  	  {
  	  		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle+4;
  	  	  }
  	  if(distance<365&&distance>170)
  	  {
  		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle+51;
  	  }
  	  else if(distance>365&&distance<530)
  	  {
  		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle;
  	  }
  	  else if(distance>530&&distance<645)
  	  {
  		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle-45;
  	  }
  	  else if(distance>645&&distance<685)
  	  {
  		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle-85;
  	  }
  	  else if(distance>685&&distance<725)
  	  {
  		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle;
  	  }
  	  else if(distance>725)
  	  {
  		  break;
  	  }
    	  }
    }






    void arrowchoice_right(void)
    {
  	  motorPidSetSpeed(0,0);
  	  MPU_Init(); //初始化MPU6050
  	  while(MPU_Init()!=0)
  	  {
  		  printf("%d",MPU_Init());
  	  };
  	  while(mpu_dmp_init()!=0)
  	  {
  	  	  printf("%d",mpu_dmp_init());

  	    };
  	  Encode1count_distance=0;
  	  Encode2count_distance=0;

  	  mpu_dmp_get_data(&pitch,&roll,&yaw);//返回�???????????????:0,DMP成功解出欧拉�???????????????
  	  while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} //这个可以解决经常读不出数据的�???????????????


  	  yaw_Rightobstacle=yaw;
  	  while(1)
    	  {
  	  distance=(Encode1count_distance+Encode2count_distance)*3.14*6.5/(4*30*11);
  	  MPU_Keeping();
  	  printf("distance:%d\r\n",distance);//显示距离
  	  if(distance<210)
  	  	  {
  	  		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle+4;
  	  	  }
  	  if(distance<360&&distance>210)
  	  {
  		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle-50;
  	  }
  	  else if(distance>360&&distance<495)
  	  {
  		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle;
  	  }
  	  else if(distance>495&&distance<595
  			  )
  	  {
  		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle+45;
  	  }
  	  else if(distance>595&&distance<640)
  	  {
  		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle+85;
  	  }
  	  else if(distance>640&&distance<675)
  	  	  {
  	  		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle;
  	  	  }
  	  else if(distance>675)
  	  {
  		  break;
  	  }
    	  }
    }



    void arrowchoice_straight(void)
    {
  	  motorPidSetSpeed(0,0);
  	  MPU_Init(); //初始化MPU6050
  	  while(MPU_Init()!=0)
  	  {
  		  printf("%d",MPU_Init());
  	  };
  	  while(mpu_dmp_init()!=0)
  	  {
  	  	  printf("%d",mpu_dmp_init());

  	    };
  	  Encode1count_distance=0;
  	  Encode2count_distance=0;

  	  mpu_dmp_get_data(&pitch,&roll,&yaw);//返回�???????????????:0,DMP成功解出欧拉�???????????????
  	  while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} //这个可以解决经常读不出数据的�???????????????

  	  yaw_Rightobstacle=yaw;
  	  while(1)
    	  {
  	  distance=(Encode1count_distance+Encode2count_distance)*3.14*6.5/(4*30*11);
  	  MPU_Keeping();
  	  printf("distance:%d\r\n",distance);//显示距离
  	  if(distance<470)
  	  {
  		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle+3;
  	  }
  	  else if(distance>470)
  	  {
  		  break;
  	  }
    	  }
    }


 /* --------------------------------------------------------------------------------- arrowchoice PID - */

 /* ---------------------------------------------------------------------------------  PID - */
//     void traffic_lightjudge(void)
//     {
//    	 motorPidSetSpeed(0,0);
//	  Encode1count_distance=0;
//	  Encode2count_distance=0;
//    	 MPU_Init(); //初始化MPU6050
//    	   	  while(MPU_Init()!=0)
//    	   	  {
//    	   		  printf("%d",MPU_Init());
//    	   	  };
//    	   	  while(mpu_dmp_init()!=0)
//    	   	  {
//    	   	  	  printf("%d",mpu_dmp_init());
//
//    	   	    };
//    	 mpu_dmp_get_data(&pitch,&roll,&yaw);//返回�???????????????:0,DMP成功解出欧拉�???????????????
//    	 while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} //这个可以解决经常读不出数据的�???????????????
//
//    	 yaw_Rightobstacle=yaw;
//		  while(1)
//			  {
//		  distance=(Encode1count_distance+Encode2count_distance)*3.14*6.5/(4*30*11);
//		  MPU_Keeping();
//		  if(distance<100)
//		  {
//			  pidMPU6050YawMovement.target_val=yaw_Rightobstacle;
//		  }
//		  else if(distance>100)
//		  {
//			  break;
//		  }
//		  }
//    	 HAL_Delay(500);
//    	 motorPidSetSpeed(0,0);
//    	 while(1)
//    	 {
//    		 if(label_value==1)
//    		 {
//    			 light_judge=1;
//    			 break;
//    		 }
//    	 }
//     }




    void traffic_lightjudge(void)
    {
    	motorPidSetSpeed(0,0);
    	while(1)
    	{
    		if(label_value==1)
    		{
    			mpu_light();
    			light_judge=1;
    			break;
    		}
    	}
    }



    void mpu_light(void)
    {
  	  motorPidSetSpeed(0,0);
  	  MPU_Init(); //初始化MPU6050
  	  while(MPU_Init()!=0)
  	  {
  		  printf("%d",MPU_Init());
  	  };
  	  while(mpu_dmp_init()!=0)
  	  {
  	  	  printf("%d",mpu_dmp_init());

  	    };
  	  Encode1count_distance=0;
  	  Encode2count_distance=0;

  	  mpu_dmp_get_data(&pitch,&roll,&yaw);//返回�???????????????:0,DMP成功解出欧拉�???????????????
  	  while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} //这个可以解决经常读不出数据的�???????????????

  	  yaw_Rightobstacle=yaw;
  	  while(1)
    	  {
  	  distance=(Encode1count_distance+Encode2count_distance)*3.14*6.5/(4*30*11);
  	  MPU_Keeping();
  	  printf("distance:%d\r\n",distance);//显示距离
  	  if(distance<200)
  	  {
  		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle+9;
  	  }
  	  else if(distance>200)
  	  {
  		  break;
  	  }
    	  }
    }
  /* --------------------------------------------------------------------------------- obstaclestop PID - */


  /* --------------------------------------------------------------------------------- obstaclestop PID - */

    void obstaclestop(void)
    {
  	motorPidSetSpeed(0,0);
  	HAL_Delay(20000);
  	while(1)
  	{
  	length1 = HC_SR04_Read1();
  	length2 = HC_SR04_Read2();

  	if(length1>30&&length2>30)
  	{
  		break;
  	}
  	else
  	{
  		Rightobstacle();
  	}
  	}
    }


  /* --------------------------------------------------------------------------------- obstaclestop PID - */

  /* --------------------------------------------------------------------------------- Rightobstacle PID - */
  void Rightobstacle(void)
  {
	  motorPidSetSpeed(0,0);
	  MPU_Init(); //初始化MPU6050
	  while(MPU_Init()!=0)
	  {
		  printf("%d",MPU_Init());
	  };
	  while(mpu_dmp_init()!=0)
	  {
	  	  printf("%d",mpu_dmp_init());

	    };

	  Encode1count_distance=0;
	  Encode2count_distance=0;

	  mpu_dmp_get_data(&pitch,&roll,&yaw);//返回�???????????????:0,DMP成功解出欧拉�???????????????
	  while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} //这个可以解决经常读不出数据的�???????????????

	  yaw_Rightobstacle=yaw;
	  while(1)
  	  {
	  distance=(Encode1count_distance+Encode2count_distance)*3.14*6.5/(4*30*11);
	  MPU_Keeping();
	  printf("distance:%d\r\n",distance);//显示距离
	  if(distance<150)
	  {
		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle+83;
	  }
	  else if(distance>150&&distance<400)
	  {
		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle;
	  }
	  else if(distance>400&&distance<530)
	  {
		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle-83;
	  }
	  else if(distance>530&&distance<560)
	  {
		  pidMPU6050YawMovement.target_val=yaw_Rightobstacle;
	  }
	  else if(distance>560)
	  {
		  ffflag=1;
		  break;
	  }
  	  }
  }

  /* --------------------------------------------------------------------------------- Rightobstacle PID - */





  /* --------------------------------------------------------------------------------- obstaclestop PID - */
  	void gate_int(void)
  	{
  	HAL_GPIO_WritePin(servo_foreward_GPIO_Port, servo_foreward_Pin, GPIO_PIN_RESET);
  	HAL_Delay(950);
  	HAL_GPIO_WritePin(servo_foreward_GPIO_Port, servo_foreward_Pin, GPIO_PIN_SET);
  	HAL_Delay(1000);
  	HAL_GPIO_WritePin(servo_reverse_GPIO_Port, servo_reverse_Pin, GPIO_PIN_RESET);
  	HAL_Delay(1000);
  	HAL_GPIO_WritePin(servo_reverse_GPIO_Port, servo_reverse_Pin, GPIO_PIN_SET);
  	HAL_Delay(1000);
  	}


    void gateopen(void)
    {
    motorPidSetSpeed(0,0);
//    MPU_Init(); //初始化MPU6050
//    	  while(MPU_Init()!=0)
//    	  {
//    		  printf("%d",MPU_Init());
//    	  };
//    	  while(mpu_dmp_init()!=0)
//    	  {
//    	  	  printf("%d",mpu_dmp_init());
//
//    	    };
    HAL_Delay(500);
	Encode1count_distance=0;
	Encode2count_distance=0;

//	mpu_dmp_get_data(&pitch,&roll,&yaw);//返回�???????????????:0,DMP成功解出欧拉�???????????????
//	while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} //这个可以解决经常读不出数据的�???????????????
//	yaw_Rightobstacle=yaw;


	HAL_GPIO_WritePin(servo_foreward_GPIO_Port, servo_foreward_Pin, GPIO_PIN_RESET);
	HAL_Delay(800);
	while(1)
	{
	distance=(Encode1count_distance+Encode2count_distance)*3.14*6.5/(4*30*11);
	Lane_Keeping(Bias);
//	MPU_Keeping(yaw_Rightobstacle);
//	pidMPU6050YawMovement.target_val=yaw_Rightobstacle;
	if(distance>30&&distance<90)
	{
		HAL_GPIO_WritePin(servo_foreward_GPIO_Port, servo_foreward_Pin, GPIO_PIN_SET);
	}
	else if(distance>100&&distance<140)
	{
		HAL_GPIO_WritePin(servo_reverse_GPIO_Port, servo_reverse_Pin, GPIO_PIN_RESET);
	}
	else if(distance>140&&distance<190)
	{
		HAL_GPIO_WritePin(servo_reverse_GPIO_Port, servo_reverse_Pin, GPIO_PIN_SET);
	}

	else if(distance>190)
	{
		while(1)
		{
		motorPidSetSpeed(0,0);
		HAL_Delay(1000);
		}

	}
	}

    }
  /* --------------------------------------------------------------------------------- obstaclestop PID - */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  gate_int();
  /* --------------------------------------------------------------------------------- OpenMV Usart 1&2 Interupt + */
  HAL_UART_Receive_IT(&huart2,(void *)&usart2_rxbuff,1);
  //HAL_UART_Receive_IT(&huart1,(void *)&usart1_rxbuff,1);调试openmv
  /* --------------------------------------------------------------------------------- OpenMV Usart 1&2 Interupt - */


  /* -------------------------------------------------------------------------------------- Motor PWM Generation + */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  /* -------------------------------------------------------------------------------------- Motor PWM Generation - */


  /* --------------------------------------------------------------------------------------------- Encoder Timer + */
  HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1 | TIM_CHANNEL_2);

  HAL_TIM_Base_Start_IT(&htim1);
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);	//�????????????????????启串�????????????????????1接收中断
  /* --------------------------------------------------------------------------------------------- Encoder Timer - */
  cJSON *cJsonData ,*cJsonVlaue;


  MPU_Init(); //初始化MPU6050
  while(MPU_Init()!=0)
  {
	  printf("%d",MPU_Init());
  };
  while(mpu_dmp_init()!=0)
  {
  	  printf("%d",mpu_dmp_init());

    };

  PID_init();
  HAL_Delay(500);//延时0.5�??????????????? 6050上电稳定后初始化


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  first_turn();

//  while(1)
//  {
//	Rightobstacle();
//  }

  Encode1count_distance=0;
  Encode2count_distance=0;
  while(1)
  {
	  Lane_Keeping(Bias);
	  distance=(Encode1count_distance+Encode2count_distance)*3.14*6.5/(4*30*11);
	  if(distance>500&&distance<530)
	  {
		  label_value_tem=label_value;
	  }
	  else if(distance>580)
	  {
		  break;
	  }

  }


  if(label_value_tem==2)
  {
	  arrowchoice_left();
  }
  else if(label_value_tem==4)
  {
	  arrowchoice_right();
  }
  else if(label_value_tem==5)
  {
	  arrowchoice_straight();
  }











  while (1)
  {

	  /* ---------------------------------------------------------------------------------------------- Encoder Code + */
	  length1 = HC_SR04_Read1();
	  length2 = HC_SR04_Read2();
	  Lane_Keeping(Bias);
	  mpu_dmp_get_data(&pitch,&roll,&yaw);//返回�???????????????:0,DMP成功解出欧拉�???????????????
	  while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} //这个可以解决经常读不出数据的�???????????????


//	  MPU_Keeping();
	  if((label_value==3||label_value==6||label_value==7)&&light_judge==0)
	  {
		traffic_lightjudge();
	  }
	  else if((length1<30||length2<30)&&(ffflag==0))
	  {
		  obstaclestop();

	  }
	  else if((length1<20||length2<20)&&(ffflag==1))
	  {
		gateopen();
	  }

//










//	  ANO_DT_Send_F2(Motor1Speed*100, 2.0*100, Motor2Speed*100, 2.0*100);
//
//	  //printf("%d\r\n",Usart_WaitReasFinish());
//	  if(Usart_WaitReasFinish() == 0)//是否接收完毕
//	  	{
//	  		cJsonData  = cJSON_Parse((const char *)Usart1_ReadBuf);
//	  		//printf("1/r/n");
//	  		if(cJSON_GetObjectItem(cJsonData,"p") !=NULL)
//	  		{
//	  			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"p");
//	  		    p = cJsonVlaue->valuedouble;
//	  			pidMotor1Speed.Kp = p;
//	  			//printf("1/r/n");
//	  		}
//	  		if(cJSON_GetObjectItem(cJsonData,"i") !=NULL)
//	  		{
//	  			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"i");
//	  		    i = cJsonVlaue->valuedouble;
//	  			pidMotor1Speed.Ki = i;
//	  		}
//	  		if(cJSON_GetObjectItem(cJsonData,"d") !=NULL)
//	  		{
//	  			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"d");
//	  		    d = cJsonVlaue->valuedouble;
//	  			pidMotor1Speed.Kd = d;
//	  		}
//	  		if(cJSON_GetObjectItem(cJsonData,"a") !=NULL)
//	  		{
//
//	  			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"a");
//	  		    a = cJsonVlaue->valuedouble;
//	  			pidMotor1Speed.target_val =a;
//	  		}
//	  		if(cJsonData != NULL){
//	  		  cJSON_Delete(cJsonData);//释放空间、但是不能删除cJsonVlaue不然�???????????????????? 出现异常错误
//	  		}
//	  		memset(Usart1_ReadBuf,0,255);//清空接收buf，注意这里不能使用strlen
//	  	}
//
//	  	printf("P:%.3f  I:%.3f  D:%.3f A:%.3f\r\n",p,i,d,a);





    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */




  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ---------------------------------------------------------------------------------- OpenMV Interupt Receive + */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint16_t tem;
  if(huart->Instance==USART2)
  {
    tem=usart2_rxbuff;
    Openmv_Receive_Data(tem);
  }
HAL_UART_Receive_IT(&huart2,(void *)&usart2_rxbuff,1);
}
/* ---------------------------------------------------------------------------------- OpenMV Interupt Receive - */


/* ---------------------------------------------------------------------- HC-SR04 Ultrasonic Interupt Receive + */

/* ---------------------------------------------------------------------- HC-SR04 Ultrasonic Interupt Receive - */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
