/**
  ******************************************************************************
  * File Name          : keys.c
  * Description        : This file provides code for the key multiplex.
  * Author             : Steven Hang (github.com/adorbs)
  ******************************************************************************
  */

#include "keys.h"
#ifdef __cplusplus
 extern "C" {
#endif

void fillKeymap()
{
	uint8_t i, j;

	for (i = 0; i < NUM_COLS; i++) {
		HAL_GPIO_WritePin(keyCols[i].port, keyCols[i].pin, 1);
		for (j = 0; j < NUM_ROWS; j++) {
			if (HAL_GPIO_ReadPin(keyRows[j].port, keyRows[j].pin))
			{
				gKeymap[j] |= 1 << i;
			}
			else
			{
				gKeymap[j] &= ~(1 << i);
			}
		}
		HAL_GPIO_WritePin(keyCols[i].port, keyCols[i].pin, 0);
	}
}
#ifdef __cplusplus
}
#endif
