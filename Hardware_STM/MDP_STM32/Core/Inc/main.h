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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
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
#define OLED_SCLK_Pin GPIO_PIN_5
#define OLED_SCLK_GPIO_Port GPIOE
#define OLED_SDIN_Pin GPIO_PIN_6
#define OLED_SDIN_GPIO_Port GPIOE
#define MOTOR_A_IN2_Pin GPIO_PIN_2
#define MOTOR_A_IN2_GPIO_Port GPIOA
#define MOTOR_A_IN1_Pin GPIO_PIN_3
#define MOTOR_A_IN1_GPIO_Port GPIOA
#define MOTOR_B_IN1_Pin GPIO_PIN_4
#define MOTOR_B_IN1_GPIO_Port GPIOA
#define MOTOR_B_IN2_Pin GPIO_PIN_5
#define MOTOR_B_IN2_GPIO_Port GPIOA
#define MOTOR_B_CH1_Pin GPIO_PIN_6
#define MOTOR_B_CH1_GPIO_Port GPIOA
#define MOTOR_B_CH2_Pin GPIO_PIN_7
#define MOTOR_B_CH2_GPIO_Port GPIOA
#define OLED_RESET_Pin GPIO_PIN_7
#define OLED_RESET_GPIO_Port GPIOE
#define OLED_DC_Pin GPIO_PIN_8
#define OLED_DC_GPIO_Port GPIOE
#define SERVO_PWM_Pin GPIO_PIN_9
#define SERVO_PWM_GPIO_Port GPIOE
#define USER_BUTTON_Pin GPIO_PIN_8
#define USER_BUTTON_GPIO_Port GPIOD
#define MOTOR_A_PWM_Pin GPIO_PIN_6
#define MOTOR_A_PWM_GPIO_Port GPIOC
#define MOTOR_B_PWM_Pin GPIO_PIN_7
#define MOTOR_B_PWM_GPIO_Port GPIOC
#define MOTOR_A_CH1_Pin GPIO_PIN_15
#define MOTOR_A_CH1_GPIO_Port GPIOA
#define US_TRIG_Pin GPIO_PIN_7
#define US_TRIG_GPIO_Port GPIOD
#define MOTOR_A_CH2_Pin GPIO_PIN_3
#define MOTOR_A_CH2_GPIO_Port GPIOB
#define ICM_SCL_Pin GPIO_PIN_8
#define ICM_SCL_GPIO_Port GPIOB
#define ICM_SDA_Pin GPIO_PIN_9
#define ICM_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
