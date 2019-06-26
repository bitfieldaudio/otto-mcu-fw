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

uint8_t gKeymap[NUM_ROWS]; // 8 Rows, bits in each byte represent the column.

void fillKeymap(void);

#ifdef __cplusplus
}
#endif
#endif /*__ keys_H */
