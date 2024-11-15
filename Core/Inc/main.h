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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;
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
#define LED_BUILDIN_Pin GPIO_PIN_13
#define LED_BUILDIN_GPIO_Port GPIOC
#define LDR_SENSOR_DIGITAL_Pin GPIO_PIN_1
#define LDR_SENSOR_DIGITAL_GPIO_Port GPIOA
#define LDR_SENSOR_ANALOG_PIN_Pin GPIO_PIN_2
#define LDR_SENSOR_ANALOG_PIN_GPIO_Port GPIOA
#define IR_SENSOR_DIGITAL_Pin GPIO_PIN_3
#define IR_SENSOR_DIGITAL_GPIO_Port GPIOA
#define IR_SENSOR_ANALOG_PIN_Pin GPIO_PIN_4
#define IR_SENSOR_ANALOG_PIN_GPIO_Port GPIOA
#define DHT11_SENSOR_Pin GPIO_PIN_5
#define DHT11_SENSOR_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOA
#define BUTTON_BLUE_Pin GPIO_PIN_1
#define BUTTON_BLUE_GPIO_Port GPIOB
#define MPU_6050_SCL_Pin GPIO_PIN_10
#define MPU_6050_SCL_GPIO_Port GPIOB
#define MPU_6050_SDA_Pin GPIO_PIN_11
#define MPU_6050_SDA_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLE_SDA_Pin GPIO_PIN_7
#define OLE_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
