#pragma once

/*
 * This set of functions handle MIDI I/O
 * From/To Raspi
 * From/To USB Device
 */

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "usbd_def.h"



typedef struct{
	uint8_t byte0;
	uint8_t byte1;
	uint8_t byte2;
} MIDI_Message;

void MIDI_USB_TX_queue(MIDI_Message message);
void MIDI_USB_TX_send(USBD_HandleTypeDef* hUsbDeviceHS);

// Receive is handled in CDC callback
void MIDI_USB_RX_parse(USBD_HandleTypeDef *hUsbDeviceHS, uint32_t buffer_len_in_bytes);

void MIDI_Raspi_TX_queue(MIDI_Message message);
void MIDI_Raspi_TX_send();
void MIDI_Raspi_RX_onReceive(MIDI_Message message);
