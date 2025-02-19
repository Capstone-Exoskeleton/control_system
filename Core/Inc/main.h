/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Power_OUT1_EN_Pin GPIO_PIN_13
#define Power_OUT1_EN_GPIO_Port GPIOC
#define Power_5V_EN_Pin GPIO_PIN_14
#define Power_5V_EN_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOC
#define Power_OUT2_EN_Pin GPIO_PIN_4
#define Power_OUT2_EN_GPIO_Port GPIOC
#define USER_KEY_Pin GPIO_PIN_15
#define USER_KEY_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define HX711_GPIO_Port GPIOC
#define HX711_CLK_Pin GPIO_PIN_7
#define HX711_DATA_Pin GPIO_PIN_6

#define I2C1_SCL_Pin GPIO_PIN_9
#define I2C1_SDA_Pin GPIO_PIN_8
#define I2C1_GPIO_Port GPIOC

typedef enum{
	IDLE = 0,
	IDLE1,
	Motor_init,
	Read_gyro,
	Motor_output,
	Wait_response,
	STOP
	
} state;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
