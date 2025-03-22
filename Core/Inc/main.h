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
#define Steering_3_Pin GPIO_PIN_5
#define Steering_3_GPIO_Port GPIOE
#define Steering_4_Pin GPIO_PIN_6
#define Steering_4_GPIO_Port GPIOE
#define blue_teeth_Pin GPIO_PIN_0
#define blue_teeth_GPIO_Port GPIOA
#define blue_teethA1_Pin GPIO_PIN_1
#define blue_teethA1_GPIO_Port GPIOA
#define Encoder_Right_Pin GPIO_PIN_6
#define Encoder_Right_GPIO_Port GPIOA
#define Encoder_RightA7_Pin GPIO_PIN_7
#define Encoder_RightA7_GPIO_Port GPIOA
#define KEY1_Pin GPIO_PIN_7
#define KEY1_GPIO_Port GPIOE
#define KEY1_EXTI_IRQn EXTI9_5_IRQn
#define NRF_IRQ_Pin GPIO_PIN_9
#define NRF_IRQ_GPIO_Port GPIOE
#define Motor_Right_Pin GPIO_PIN_11
#define Motor_Right_GPIO_Port GPIOE
#define Motor_Left_Pin GPIO_PIN_14
#define Motor_Left_GPIO_Port GPIOE
#define openmv_Pin GPIO_PIN_8
#define openmv_GPIO_Port GPIOD
#define openmvD9_Pin GPIO_PIN_9
#define openmvD9_GPIO_Port GPIOD
#define NRF_CS_Pin GPIO_PIN_10
#define NRF_CS_GPIO_Port GPIOD
#define Key_2_Pin GPIO_PIN_11
#define Key_2_GPIO_Port GPIOD
#define Encoder_Left_Pin GPIO_PIN_12
#define Encoder_Left_GPIO_Port GPIOD
#define Encoder_LeftD13_Pin GPIO_PIN_13
#define Encoder_LeftD13_GPIO_Port GPIOD
#define NRF_CE_Pin GPIO_PIN_15
#define NRF_CE_GPIO_Port GPIOD
#define jy901_Pin GPIO_PIN_6
#define jy901_GPIO_Port GPIOC
#define jy901C7_Pin GPIO_PIN_7
#define jy901C7_GPIO_Port GPIOC
#define wht101_Pin GPIO_PIN_5
#define wht101_GPIO_Port GPIOD
#define wht101D6_Pin GPIO_PIN_6
#define wht101D6_GPIO_Port GPIOD
#define Steering_1_Pin GPIO_PIN_8
#define Steering_1_GPIO_Port GPIOB
#define Steering_2_Pin GPIO_PIN_9
#define Steering_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
