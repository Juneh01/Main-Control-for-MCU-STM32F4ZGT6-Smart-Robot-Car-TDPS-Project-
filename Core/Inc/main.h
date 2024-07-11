/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Rightobstacle_inter_Pin GPIO_PIN_13
#define Rightobstacle_inter_GPIO_Port GPIOC
#define Rightobstacle_inter_EXTI_IRQn EXTI15_10_IRQn
#define LED0_Pin GPIO_PIN_9
#define LED0_GPIO_Port GPIOF
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOF
#define HC_SR04_Echo1_Pin GPIO_PIN_6
#define HC_SR04_Echo1_GPIO_Port GPIOA
#define HC_SR04_Trig1_Pin GPIO_PIN_7
#define HC_SR04_Trig1_GPIO_Port GPIOA
#define SCL_6050_Pin GPIO_PIN_12
#define SCL_6050_GPIO_Port GPIOB
#define SDA_6050_Pin GPIO_PIN_13
#define SDA_6050_GPIO_Port GPIOB
#define servo_foreward_Pin GPIO_PIN_6
#define servo_foreward_GPIO_Port GPIOG
#define servo_reverse_Pin GPIO_PIN_7
#define servo_reverse_GPIO_Port GPIOG
#define PWMB_Pin GPIO_PIN_8
#define PWMB_GPIO_Port GPIOA
#define PWMA_Pin GPIO_PIN_11
#define PWMA_GPIO_Port GPIOA
#define HC_SR04_Echo2_Pin GPIO_PIN_3
#define HC_SR04_Echo2_GPIO_Port GPIOD
#define HC_SR04_Trig2_Pin GPIO_PIN_4
#define HC_SR04_Trig2_GPIO_Port GPIOD
#define BIN2_Pin GPIO_PIN_4
#define BIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_5
#define BIN1_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_6
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_7
#define AIN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
