/**
  ******************************************************************************
  * File Name          : otto.h
  * Description        : This file is the header file for board specific code.
  * Author             : Steven Hang (github.com/adorbs)
  ******************************************************************************
  */
#ifndef __otto_H
#define __otto_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"

typedef struct pindef_t
{
	GPIO_TypeDef* port;
	uint16_t pin;
}pindef_t;

#define NUM_COLS		7
#define GPIO_COL_1		(pindef_t){GPIOA,GPIO_PIN_9}
#define GPIO_COL_2		(pindef_t){GPIOC,GPIO_PIN_10}
#define GPIO_COL_3		(pindef_t){GPIOB,GPIO_PIN_5}
#define GPIO_COL_4		(pindef_t){GPIOA,GPIO_PIN_15}
#define GPIO_COL_5		(pindef_t){GPIOC,GPIO_PIN_0}
#define GPIO_COL_6		(pindef_t){GPIOC,GPIO_PIN_1}
#define GPIO_COL_7		(pindef_t){GPIOC,GPIO_PIN_3}

#define NUM_ROWS		8
#define GPIO_ROW_1		(pindef_t){GPIOC,GPIO_PIN_11}
#define GPIO_ROW_2		(pindef_t){GPIOC,GPIO_PIN_2}
#define GPIO_ROW_3		(pindef_t){GPIOB,GPIO_PIN_4}
#define GPIO_ROW_4		(pindef_t){GPIOA,GPIO_PIN_10}
#define GPIO_ROW_5		(pindef_t){GPIOB,GPIO_PIN_3}
#define GPIO_ROW_6		(pindef_t){GPIOD,GPIO_PIN_2}
#define GPIO_ROW_7		(pindef_t){GPIOA,GPIO_PIN_8}
#define GPIO_ROW_8		(pindef_t){GPIOC,GPIO_PIN_12}

#define NUM_LED_STRINGS 3
#define GPIO_SPI_CS_MCU_LED_FUNC	(pindef_t){GPIOA,GPIO_PIN_4}
#define GPIO_SPI_CS_MCU_LED_DRUM	(pindef_t){GPIOC,GPIO_PIN_4}
#define GPIO_SPI_CS_MCU_LED_SEQ		(pindef_t){GPIOC,GPIO_PIN_5}

#define NUM_ENCODERS	4
#define GPIO_ENC_A_1	(pindef_t){GPIOC, GPIO_PIN_6}
#define GPIO_ENC_B_1	(pindef_t){GPIOC, GPIO_PIN_8}
#define GPIO_ENC_A_2	(pindef_t){GPIOC, GPIO_PIN_7}
#define GPIO_ENC_B_2	(pindef_t){GPIOB, GPIO_PIN_14}
#define GPIO_ENC_A_3	(pindef_t){GPIOA, GPIO_PIN_3}
#define GPIO_ENC_B_3	(pindef_t){GPIOA, GPIO_PIN_2}
#define GPIO_ENC_A_4	(pindef_t){GPIOA, GPIO_PIN_1}
#define GPIO_ENC_B_4	(pindef_t){GPIOA, GPIO_PIN_0}

#define GPIO_LED_PIN	(pindef_t){GPIOB, GPIO_PIN_11}

#define OTTO_I2C_ADDRESS 0x77<<1

#ifdef __cplusplus
}
#endif
#endif /*__otto_H */
