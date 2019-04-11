/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32mp1xx_hal.h"

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
#define BMP388_CS_Pin GPIO_PIN_0
#define BMP388_CS_GPIO_Port GPIOI
#define BMA456_INT1_Pin GPIO_PIN_15
#define BMA456_INT1_GPIO_Port GPIOD
#define BMA456_INT1_EXTI_IRQn EXTI15_IRQn
#define LED_TEST_Pin GPIO_PIN_7
#define LED_TEST_GPIO_Port GPIOZ
#define BMA456_CS_Pin GPIO_PIN_8
#define BMA456_CS_GPIO_Port GPIOD
#define BHI160_INT_Pin GPIO_PIN_14
#define BHI160_INT_GPIO_Port GPIOA
#define BHI160_INT_EXTI_IRQn EXTI14_IRQn
#define LED_ERROR_Pin GPIO_PIN_1
#define LED_ERROR_GPIO_Port GPIOG
#define BME680_CS_Pin GPIO_PIN_3
#define BME680_CS_GPIO_Port GPIOC
#define LED_STATUS_Pin GPIO_PIN_0
#define LED_STATUS_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
