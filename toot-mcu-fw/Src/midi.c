#include "midi.h"
#include "usbd_cdc_if.h"


#define midi_USB_TX_buffer_size 64
uint8_t midi_USB_TX_buffer[midi_USB_TX_buffer_size];
uint8_t midi_USB_TX_index = 0;

#define midi_USB_RX_buffer_size 64
MIDI_Message midi_USB_RX_buffer[midi_USB_RX_buffer_size];
uint8_t midi_USB_RX_index = 0;

#define MIDI_RASPI_TX_BUFFER_SIZE 64
MIDI_Message midi_raspi_TX_buffer[MIDI_RASPI_TX_BUFFER_SIZE];
uint8_t midi_raspi_TX_index = 0;

void MIDI_USB_TX_queue(MIDI_Message message) {
	midi_USB_TX_buffer[midi_USB_TX_index++] = message.byte0 >> 4;
	midi_USB_TX_buffer[midi_USB_TX_index++] = message.byte0;
	midi_USB_TX_buffer[midi_USB_TX_index++] = message.byte1;
	midi_USB_TX_buffer[midi_USB_TX_index++] = message.byte2;
}

void MIDI_USB_TX_send(USBD_HandleTypeDef* hUsbDeviceHS){
	if(midi_USB_TX_index == 0) return;

	USBD_CDC_SetTxBuffer(hUsbDeviceHS, midi_USB_TX_buffer, midi_USB_TX_index);
	USBD_CDC_TransmitPacket(hUsbDeviceHS);
	midi_USB_TX_index = 0;
}


void MIDI_USB_RX_parse(USBD_HandleTypeDef *hUsbDeviceHS, uint32_t buffer_len_in_bytes) {
  //each USB midi package is 4 bytes long
  // The first is unused, the three other contain data
  uint16_t messages_size = buffer_len_in_bytes / 4;
  uint8_t *ptr = UserRxBufferFS;

  MIDI_Message message;

  while(messages_size--)
	{
	  ptr++ ; // discard first byte
	  message.byte0 = *ptr ; ptr++ ;
	  message.byte1 = *ptr ; ptr++ ;
	  message.byte2 = *ptr ; ptr++ ;
	  MIDI_Raspi_TX_queue(message);
	}
}

void MIDI_Raspi_TX_queue(MIDI_Message message){
	midi_raspi_TX_buffer[midi_raspi_TX_index++] = message;
}
void MIDI_Raspi_TX_send(){
	//TODO
	midi_raspi_TX_index = 0;
}

void MIDI_Raspi_RX_onReceive(MIDI_Message message){
	MIDI_USB_TX_queue(message);
}
