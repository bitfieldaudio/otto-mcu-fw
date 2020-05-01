/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

extern ADC_HandleTypeDef hadc1;

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi3;

extern UART_HandleTypeDef huart3;

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USE_HAL_I2C_REGISTER_CALLBACKS 1
#define ROW_5_Pin GPIO_PIN_0
#define ROW_5_GPIO_Port GPIOC
#define ROW_6_Pin GPIO_PIN_1
#define ROW_6_GPIO_Port GPIOC
#define ADC_VERSION_Pin GPIO_PIN_2
#define ADC_VERSION_GPIO_Port GPIOC
#define ROW_8_Pin GPIO_PIN_3
#define ROW_8_GPIO_Port GPIOC
#define DC_IN_DIV_5_Pin GPIO_PIN_0
#define DC_IN_DIV_5_GPIO_Port GPIOA
#define RASPI_3V3_DIV_2_Pin GPIO_PIN_2
#define RASPI_3V3_DIV_2_GPIO_Port GPIOA
#define REG_5V_IOUT_Pin GPIO_PIN_3
#define REG_5V_IOUT_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define COL_3_Pin GPIO_PIN_7
#define COL_3_GPIO_Port GPIOE
#define COL_4_Pin GPIO_PIN_8
#define COL_4_GPIO_Port GPIOE
#define COL_5_Pin GPIO_PIN_10
#define COL_5_GPIO_Port GPIOE
#define COL_6_Pin GPIO_PIN_12
#define COL_6_GPIO_Port GPIOE
#define COL_7_Pin GPIO_PIN_15
#define COL_7_GPIO_Port GPIOE
#define ROW_1_Pin GPIO_PIN_12
#define ROW_1_GPIO_Port GPIOB
#define ROW_2_Pin GPIO_PIN_13
#define ROW_2_GPIO_Port GPIOB
#define ROW_3_Pin GPIO_PIN_14
#define ROW_3_GPIO_Port GPIOB
#define ROW_4_Pin GPIO_PIN_15
#define ROW_4_GPIO_Port GPIOB
#define ENC_3_A_Pin GPIO_PIN_8
#define ENC_3_A_GPIO_Port GPIOD
#define ENC_3_A_EXTI_IRQn EXTI9_5_IRQn
#define ENC_3_B_Pin GPIO_PIN_9
#define ENC_3_B_GPIO_Port GPIOD
#define ENC_3_B_EXTI_IRQn EXTI9_5_IRQn
#define ENC_4_A_Pin GPIO_PIN_10
#define ENC_4_A_GPIO_Port GPIOD
#define ENC_4_A_EXTI_IRQn EXTI15_10_IRQn
#define ENC_4_B_Pin GPIO_PIN_11
#define ENC_4_B_GPIO_Port GPIOD
#define ENC_4_B_EXTI_IRQn EXTI15_10_IRQn
#define TRIGGER1_Pin GPIO_PIN_12
#define TRIGGER1_GPIO_Port GPIOD
#define TRIGGER1_EXTI_IRQn EXTI15_10_IRQn
#define TRIGGER2_Pin GPIO_PIN_13
#define TRIGGER2_GPIO_Port GPIOD
#define TRIGGER2_EXTI_IRQn EXTI15_10_IRQn
#define TRIGGER3_Pin GPIO_PIN_14
#define TRIGGER3_GPIO_Port GPIOD
#define TRIGGER3_EXTI_IRQn EXTI15_10_IRQn
#define PWM1_GPIO13_RASPI_Pin GPIO_PIN_15
#define PWM1_GPIO13_RASPI_GPIO_Port GPIOD
#define PWM1_GPIO13_RASPI_EXTI_IRQn EXTI15_10_IRQn
#define STATUS_LED_Pin GPIO_PIN_6
#define STATUS_LED_GPIO_Port GPIOC
#define REG_5V_PWM_Pin GPIO_PIN_7
#define REG_5V_PWM_GPIO_Port GPIOC
#define TEST_POINT_1_Pin GPIO_PIN_8
#define TEST_POINT_1_GPIO_Port GPIOC
#define TEST_POINT_1_EXTI_IRQn EXTI9_5_IRQn
#define LED_PWR_EN_Pin GPIO_PIN_10
#define LED_PWR_EN_GPIO_Port GPIOA
#define COL_8_Pin GPIO_PIN_10
#define COL_8_GPIO_Port GPIOC
#define ROW_7_Pin GPIO_PIN_12
#define ROW_7_GPIO_Port GPIOC
#define PWR_BUTTON_Pin GPIO_PIN_0
#define PWR_BUTTON_GPIO_Port GPIOD
#define PI_PWR_EN_Pin GPIO_PIN_1
#define PI_PWR_EN_GPIO_Port GPIOD
#define ENC_1_A_Pin GPIO_PIN_2
#define ENC_1_A_GPIO_Port GPIOD
#define ENC_1_A_EXTI_IRQn EXTI2_IRQn
#define ENC_1_B_Pin GPIO_PIN_3
#define ENC_1_B_GPIO_Port GPIOD
#define ENC_1_B_EXTI_IRQn EXTI3_IRQn
#define ENC_2_A_Pin GPIO_PIN_4
#define ENC_2_A_GPIO_Port GPIOD
#define ENC_2_A_EXTI_IRQn EXTI4_IRQn
#define MIDI_OUT_Pin GPIO_PIN_5
#define MIDI_OUT_GPIO_Port GPIOD
#define MIDI_IN_Pin GPIO_PIN_6
#define MIDI_IN_GPIO_Port GPIOD
#define MIDI_IN_EXTI_IRQn EXTI9_5_IRQn
#define ENC_2_B_Pin GPIO_PIN_7
#define ENC_2_B_GPIO_Port GPIOD
#define ENC_2_B_EXTI_IRQn EXTI9_5_IRQn
#define GPIO1_RASPI_Pin GPIO_PIN_7
#define GPIO1_RASPI_GPIO_Port GPIOB
#define COL_1_Pin GPIO_PIN_0
#define COL_1_GPIO_Port GPIOE
#define COL_2_Pin GPIO_PIN_1
#define COL_2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
