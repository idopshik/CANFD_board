/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Button_3_Pin GPIO_PIN_0
#define Button_3_GPIO_Port GPIOA
#define Button_2_Pin GPIO_PIN_1
#define Button_2_GPIO_Port GPIOA
#define Button_1_Pin GPIO_PIN_2
#define Button_1_GPIO_Port GPIOA
#define LED_TX1_Pin GPIO_PIN_3
#define LED_TX1_GPIO_Port GPIOA
#define LED_RX1_Pin GPIO_PIN_4
#define LED_RX1_GPIO_Port GPIOA
#define LED_TX2_Pin GPIO_PIN_5
#define LED_TX2_GPIO_Port GPIOA
#define LED_RX2_Pin GPIO_PIN_6
#define LED_RX2_GPIO_Port GPIOA
#define LED_TX3_Pin GPIO_PIN_7
#define LED_TX3_GPIO_Port GPIOA
#define LED_RX3_Pin GPIO_PIN_4
#define LED_RX3_GPIO_Port GPIOC
#define CAN_RX2_Pin GPIO_PIN_12
#define CAN_RX2_GPIO_Port GPIOB
#define CAN_TX2_Pin GPIO_PIN_13
#define CAN_TX2_GPIO_Port GPIOB
#define CAN_RX3_Pin GPIO_PIN_8
#define CAN_RX3_GPIO_Port GPIOA
#define CAN_RX1_Pin GPIO_PIN_11
#define CAN_RX1_GPIO_Port GPIOA
#define CAN_TX1_Pin GPIO_PIN_12
#define CAN_TX1_GPIO_Port GPIOA
#define Pre_pre_Pin GPIO_PIN_7
#define Pre_pre_GPIO_Port GPIOB
#define PreLast_Pin GPIO_PIN_9
#define PreLast_GPIO_Port GPIOB
#define LastPin_Pin GPIO_PIN_0
#define LastPin_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
