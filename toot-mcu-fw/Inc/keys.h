/**
  ******************************************************************************
  * File Name          : keys.h
  * Description        : This file is the header file for the key multiplex code.
  * Author             : Steven Hang (github.com/adorbs)
  ******************************************************************************
  */
#ifndef __keys_H
#define __keys_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "toot.h"

static const pindef_t keyCols[] = {GPIO_COL_1, GPIO_COL_2, GPIO_COL_3,
			   GPIO_COL_4, GPIO_COL_5, GPIO_COL_6,
			   GPIO_COL_7};

static const pindef_t keyRows[] = {GPIO_ROW_1, GPIO_ROW_2, GPIO_ROW_3,
				GPIO_ROW_4, GPIO_ROW_5, GPIO_ROW_6,
				GPIO_ROW_7, GPIO_ROW_8};

volatile uint8_t gKeymap[NUM_ROWS]; // 8 Rows, bits in each byte represent the column.

void fillKeymap(void);

#ifdef __cplusplus
}
#endif
#endif /*__ keys_H */
