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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AIR_EBS_Pin GPIO_PIN_0
#define AIR_EBS_GPIO_Port GPIOC
#define AIR_RDNT_Pin GPIO_PIN_1
#define AIR_RDNT_GPIO_Port GPIOC
#define AIR_MAIN_Pin GPIO_PIN_2
#define AIR_MAIN_GPIO_Port GPIOC
#define LD_OK_Pin GPIO_PIN_0
#define LD_OK_GPIO_Port GPIOA
#define LD_WARN_Pin GPIO_PIN_1
#define LD_WARN_GPIO_Port GPIOA
#define LD_ERROR_Pin GPIO_PIN_2
#define LD_ERROR_GPIO_Port GPIOA
#define LD_CHK_Pin GPIO_PIN_3
#define LD_CHK_GPIO_Port GPIOA
#define AS_CLOSE_SDC_Pin GPIO_PIN_5
#define AS_CLOSE_SDC_GPIO_Port GPIOA
#define AS_MODE_Pin GPIO_PIN_6
#define AS_MODE_GPIO_Port GPIOA
#define spare1_Pin GPIO_PIN_0
#define spare1_GPIO_Port GPIOB
#define spare2_Pin GPIO_PIN_1
#define spare2_GPIO_Port GPIOB
#define VALVE1_Pin GPIO_PIN_11
#define VALVE1_GPIO_Port GPIOB
#define VALVE2_Pin GPIO_PIN_13
#define VALVE2_GPIO_Port GPIOB
#define SDC_RDY_Pin GPIO_PIN_7
#define SDC_RDY_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
