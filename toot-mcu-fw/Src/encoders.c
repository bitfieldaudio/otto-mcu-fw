/**
  ******************************************************************************
  * File Name          : encoders.c
  * Description        : This file provides code for the encoders.
  * Author             : Steven Hang (github.com/adorbs)
  ******************************************************************************
  */

#include "encoders.h"

void initEncoder(encoder_t* enc, pindef_t pin_a, pindef_t pin_b)
{
	enc->pinA = pin_a;
	enc->pinB = pin_b;
	enc->value = 0;
}
