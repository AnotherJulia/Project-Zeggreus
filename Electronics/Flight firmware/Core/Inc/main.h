/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VOLTAGE_Pin GPIO_PIN_2
#define VOLTAGE_GPIO_Port GPIOC
#define RBF_Pin GPIO_PIN_0
#define RBF_GPIO_Port GPIOA
#define SD_NSS_Pin GPIO_PIN_4
#define SD_NSS_GPIO_Port GPIOC
#define DEBUG_Pin GPIO_PIN_5
#define DEBUG_GPIO_Port GPIOC
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define LORA_NSS_Pin GPIO_PIN_15
#define LORA_NSS_GPIO_Port GPIOA
#define LORA_DIO1_Pin GPIO_PIN_12
#define LORA_DIO1_GPIO_Port GPIOC
#define LORA_BUSY_Pin GPIO_PIN_2
#define LORA_BUSY_GPIO_Port GPIOD
#define SD_DETECT_Pin GPIO_PIN_4
#define SD_DETECT_GPIO_Port GPIOB
#define LORA_RESET_Pin GPIO_PIN_6
#define LORA_RESET_GPIO_Port GPIOB
#define BREAKWIRE_Pin GPIO_PIN_8
#define BREAKWIRE_GPIO_Port GPIOB
#define ARM_Pin GPIO_PIN_9
#define ARM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
