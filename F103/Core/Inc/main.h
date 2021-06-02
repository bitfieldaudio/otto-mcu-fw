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
#define PWM1_GPIO13_RASPI_Pin GPIO_PIN_13
#define PWM1_GPIO13_RASPI_GPIO_Port GPIOC
#define ENC_A_4_Pin GPIO_PIN_0
#define ENC_A_4_GPIO_Port GPIOC
#define TRIGGER1_Pin GPIO_PIN_1
#define TRIGGER1_GPIO_Port GPIOC
#define TRIGGER2_Pin GPIO_PIN_2
#define TRIGGER2_GPIO_Port GPIOC
#define TRIGGER3_Pin GPIO_PIN_3
#define TRIGGER3_GPIO_Port GPIOC
#define ENC_B_3_Pin GPIO_PIN_0
#define ENC_B_3_GPIO_Port GPIOA
#define ENC_A_3_Pin GPIO_PIN_1
#define ENC_A_3_GPIO_Port GPIOA
#define COL_3_Pin GPIO_PIN_6
#define COL_3_GPIO_Port GPIOA
#define COL_4_Pin GPIO_PIN_4
#define COL_4_GPIO_Port GPIOC
#define COL_5_Pin GPIO_PIN_5
#define COL_5_GPIO_Port GPIOC
#define COL_2_Pin GPIO_PIN_0
#define COL_2_GPIO_Port GPIOB
#define COL_7_Pin GPIO_PIN_1
#define COL_7_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define COL_6_Pin GPIO_PIN_10
#define COL_6_GPIO_Port GPIOB
#define COL_1_Pin GPIO_PIN_11
#define COL_1_GPIO_Port GPIOB
#define ROW_2_Pin GPIO_PIN_12
#define ROW_2_GPIO_Port GPIOB
#define ROW_3_Pin GPIO_PIN_13
#define ROW_3_GPIO_Port GPIOB
#define ROW_4_Pin GPIO_PIN_14
#define ROW_4_GPIO_Port GPIOB
#define ROW_5_Pin GPIO_PIN_15
#define ROW_5_GPIO_Port GPIOB
#define STATUS_LED_Pin GPIO_PIN_6
#define STATUS_LED_GPIO_Port GPIOC
#define ENC_B_1_Pin GPIO_PIN_7
#define ENC_B_1_GPIO_Port GPIOC
#define ENC_A_1_Pin GPIO_PIN_8
#define ENC_A_1_GPIO_Port GPIOC
#define ROW_1_Pin GPIO_PIN_9
#define ROW_1_GPIO_Port GPIOC
#define LED_PWR_EN_Pin GPIO_PIN_8
#define LED_PWR_EN_GPIO_Port GPIOA
#define ENC_B_4_Pin GPIO_PIN_15
#define ENC_B_4_GPIO_Port GPIOA
#define PWR_HOLD_Pin GPIO_PIN_10
#define PWR_HOLD_GPIO_Port GPIOC
#define PWR_BUTTON_Pin GPIO_PIN_11
#define PWR_BUTTON_GPIO_Port GPIOC
#define ENC_A_2_Pin GPIO_PIN_12
#define ENC_A_2_GPIO_Port GPIOC
#define GPIO1_RASPI_Pin GPIO_PIN_2
#define GPIO1_RASPI_GPIO_Port GPIOD
#define COL_8_Pin GPIO_PIN_3
#define COL_8_GPIO_Port GPIOB
#define ROW_7_Pin GPIO_PIN_4
#define ROW_7_GPIO_Port GPIOB
#define ENC_B_2_Pin GPIO_PIN_5
#define ENC_B_2_GPIO_Port GPIOB
#define ROW_6_Pin GPIO_PIN_8
#define ROW_6_GPIO_Port GPIOB
#define ROW_8_Pin GPIO_PIN_9
#define ROW_8_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
