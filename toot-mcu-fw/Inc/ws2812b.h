/**
  ******************************************************************************
  * File Name          : ws2812b.h
  * Description        : This file is the header file for the ws2812b control code.
  * Author             : Steven Hang (github.com/adorbs)
  ******************************************************************************
  */
#ifndef __ws2812b_H
#define __ws2812b_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "toot.h"
#include <stdlib.h>

//256*3 byte array to encode SPI stream for WS2812B
uint8_t encoderLookup[768];

typedef struct ws2812b_t
{
	SPI_HandleTypeDef* spiHandle;
	pindef_t spiCS;
	uint16_t numPixels;
	uint16_t numBytes;
	uint8_t* pixelDataArray;
} ws2812b_t;

void initWS2812B(ws2812b_t* string, SPI_HandleTypeDef* spiHandle, pindef_t spiCS, uint16_t numPixels);
void setPixelColor(ws2812b_t* string, uint16_t n, uint8_t r, uint8_t g, uint8_t b);
void setPixelColorHex(ws2812b_t* string, uint16_t n, uint32_t c);
void clearPixels(ws2812b_t* string);
void showPixels(ws2812b_t* string);

#ifdef __cplusplus
}
#endif
#endif /*__ ws2812b_H */
