/**
  ******************************************************************************
  * File Name          : keys.c
  * Description        : This file provides code for the key multiplex.
  * Author             : Steven Hang (github.com/adorbs)
  ******************************************************************************
  */

#include "keys.h"

void fillKeymap()
{
	pindef_t cols[] = {GPIO_COL_1, GPIO_COL_2, GPIO_COL_3,
				   GPIO_COL_4, GPIO_COL_5, GPIO_COL_6,
				   GPIO_COL_7};

	pindef_t rows[] = {GPIO_ROW_1, GPIO_ROW_2, GPIO_ROW_3,
					GPIO_ROW_4, GPIO_ROW_5, GPIO_ROW_6,
					GPIO_ROW_7, GPIO_ROW_8};
	uint8_t i, j;

	for (i = 0; i < NUM_COLS; i++) {
		HAL_GPIO_WritePin(cols[i].port, cols[i].pin, 1);
		for (j = 0; j < NUM_ROWS; j++) {
			if (HAL_GPIO_ReadPin(rows[j].port, rows[j].pin))
			{
				gKeymap[j] |= 1 << i;
			}
			else
			{
				gKeymap[j] &= ~(1 << i);
			}
		}
		HAL_GPIO_WritePin(cols[i].port, cols[i].pin, 0);
	}
}
