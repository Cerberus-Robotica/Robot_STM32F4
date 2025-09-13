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
#define LED_AZUL_Pin GPIO_PIN_13
#define LED_AZUL_GPIO_Port GPIOC
#define SPI_CSN_Pin GPIO_PIN_3
#define SPI_CSN_GPIO_Port GPIOA
#define SPI_CE_Pin GPIO_PIN_4
#define SPI_CE_GPIO_Port GPIOA
#define motor4_in3_PWM1_Pin GPIO_PIN_0
#define motor4_in3_PWM1_GPIO_Port GPIOB
#define motor3_in2_PWM2_Pin GPIO_PIN_1
#define motor3_in2_PWM2_GPIO_Port GPIOB
#define sinal_kicker_Pin GPIO_PIN_2
#define sinal_kicker_GPIO_Port GPIOB
#define motor3_in1_PWM1_Pin GPIO_PIN_10
#define motor3_in1_PWM1_GPIO_Port GPIOB
#define sinal_hall_1_Pin GPIO_PIN_12
#define sinal_hall_1_GPIO_Port GPIOB
#define sinal_hall_2_Pin GPIO_PIN_13
#define sinal_hall_2_GPIO_Port GPIOB
#define sinal_hall_3_Pin GPIO_PIN_14
#define sinal_hall_3_GPIO_Port GPIOB
#define sinal_hall_4_Pin GPIO_PIN_15
#define sinal_hall_4_GPIO_Port GPIOB
#define motor1_IN2_PWM2_Pin GPIO_PIN_8
#define motor1_IN2_PWM2_GPIO_Port GPIOA
#define motor1_IN1_PWM1_Pin GPIO_PIN_9
#define motor1_IN1_PWM1_GPIO_Port GPIOA
#define motor2_IN3_PWM1_Pin GPIO_PIN_10
#define motor2_IN3_PWM1_GPIO_Port GPIOA
#define motor2_in4_PWM2_Pin GPIO_PIN_11
#define motor2_in4_PWM2_GPIO_Port GPIOA
#define Sensor_bola_Pin GPIO_PIN_12
#define Sensor_bola_GPIO_Port GPIOA
#define motor4_in4_PWM2_Pin GPIO_PIN_15
#define motor4_in4_PWM2_GPIO_Port GPIOA
#define sinal_hall_5_Pin GPIO_PIN_3
#define sinal_hall_5_GPIO_Port GPIOB
#define sinal_hall_6_Pin GPIO_PIN_4
#define sinal_hall_6_GPIO_Port GPIOB
#define sinal_hall_7_Pin GPIO_PIN_5
#define sinal_hall_7_GPIO_Port GPIOB
#define sinal_hall_8_Pin GPIO_PIN_8
#define sinal_hall_8_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
