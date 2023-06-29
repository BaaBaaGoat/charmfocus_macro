/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define GPIO_GAMEPAD_TOUCH_Pin GPIO_PIN_0
#define GPIO_GAMEPAD_TOUCH_GPIO_Port GPIOA
#define GPIO_GAMEPAD_R1_Pin GPIO_PIN_1
#define GPIO_GAMEPAD_R1_GPIO_Port GPIOA
#define GPIO_GAMEPAD_UP_Pin GPIO_PIN_2
#define GPIO_GAMEPAD_UP_GPIO_Port GPIOA
#define GPIO_GAMEPAD_LEFT_Pin GPIO_PIN_3
#define GPIO_GAMEPAD_LEFT_GPIO_Port GPIOA
#define GPIO_GAMEPAD_DOWN_Pin GPIO_PIN_4
#define GPIO_GAMEPAD_DOWN_GPIO_Port GPIOA
#define GPIO_GAMEPAD_RIGHT_Pin GPIO_PIN_5
#define GPIO_GAMEPAD_RIGHT_GPIO_Port GPIOA
#define GPIO_GAMEPAD_TRIANGLE_Pin GPIO_PIN_2
#define GPIO_GAMEPAD_TRIANGLE_GPIO_Port GPIOB
#define LED_A_PLUS_B_Pin GPIO_PIN_10
#define LED_A_PLUS_B_GPIO_Port GPIOB
#define LED_FIST_Pin GPIO_PIN_11
#define LED_FIST_GPIO_Port GPIOB
#define LED_BULLET_Pin GPIO_PIN_13
#define LED_BULLET_GPIO_Port GPIOB
#define LED_BULLET_A_Pin GPIO_PIN_14
#define LED_BULLET_A_GPIO_Port GPIOB
#define LED_USB_Pin GPIO_PIN_15
#define LED_USB_GPIO_Port GPIOB
#define GPIO_GAMEPAD_SQUARE_Pin GPIO_PIN_8
#define GPIO_GAMEPAD_SQUARE_GPIO_Port GPIOA
#define GPIO_GAMEPAD_X_Pin GPIO_PIN_9
#define GPIO_GAMEPAD_X_GPIO_Port GPIOA
#define GPIO_GAMEPAD_CIRCLE_Pin GPIO_PIN_10
#define GPIO_GAMEPAD_CIRCLE_GPIO_Port GPIOA
#define GPIO_GAMEPAD_L1_Pin GPIO_PIN_15
#define GPIO_GAMEPAD_L1_GPIO_Port GPIOA
#define BACKKEY1_Pin GPIO_PIN_4
#define BACKKEY1_GPIO_Port GPIOB
#define BACKKEY3_Pin GPIO_PIN_5
#define BACKKEY3_GPIO_Port GPIOB
#define BACKKEY2_Pin GPIO_PIN_6
#define BACKKEY2_GPIO_Port GPIOB
#define BACKKEY4_Pin GPIO_PIN_7
#define BACKKEY4_GPIO_Port GPIOB
#define BACKKEY_LEFT_Pin GPIO_PIN_8
#define BACKKEY_LEFT_GPIO_Port GPIOB
#define BACKKEY_RIGHT_Pin GPIO_PIN_9
#define BACKKEY_RIGHT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
