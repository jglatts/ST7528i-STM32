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

/* Includes */
#include "stm32f1xx_hal.h"

/* Exported functions prototypes */
void SystemClock_Config(void);
void Error_Handler(void);

/* Private defines */
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define RST_Pin GPIO_PIN_14
#define RST_GPIO_Port GPIOC


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
