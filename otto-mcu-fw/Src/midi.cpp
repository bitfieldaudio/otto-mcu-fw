#include "midi.h"
#include "commands.h"
#include "usbd_cdc_if.h"
#include "ws2812b.h"

#include <assert.h>

#define midi_USB_TX_buffer_size 64
uint8_t midi_USB_TX_buffer[midi_USB_TX_buffer_size];
volatile uint8_t midi_USB_TX_index = 0;

#define midi_USB_RX_buffer_size 64
MIDI_Message midi_USB_RX_buffer[midi_USB_RX_buffer_size];
volatile uint8_t midi_USB_RX_index = 0;

#define MIDI_RASPI_TX_BUFFER_SIZE 64
MIDI_Message midi_raspi_TX_buffer[MIDI_RASPI_TX_BUFFER_SIZE];
volatile uint8_t midi_raspi_TX_index = 0;

void MIDI_USB_TX_queue(MIDI_Message message)
{
  // https://www.usb.org/sites/default/files/midi10.pdf page 16
  midi_USB_TX_buffer[midi_USB_TX_index++] = message.byte0 >> 4;
  midi_USB_TX_buffer[midi_USB_TX_index++] = message.byte0;
  midi_USB_TX_buffer[midi_USB_TX_index++] = message.byte1;
  midi_USB_TX_buffer[midi_USB_TX_index++] = message.byte2;
}

void MIDI_USB_TX_send(USBD_HandleTypeDef* hUsbDeviceHS)
{
  if (midi_USB_TX_index == 0) return;

  USBD_CDC_SetTxBuffer(hUsbDeviceHS, midi_USB_TX_buffer, midi_USB_TX_index);
  USBD_CDC_TransmitPacket(hUsbDeviceHS);
  midi_USB_TX_index = 0;
}

static SlaveState midi_slave_state;
#define MIDI_SYSEX_buffer_size 64
uint8_t midi_SYSEX_buffer[MIDI_SYSEX_buffer_size];
volatile int8_t midi_SYSEX_index = -1;

#define SYSEX_START_BYTE 0xF0
#define SYSEX_END_BYTE 0xF7

// Return true if done
static bool append_midi_sysex(uint8_t byte)
{
  if (byte == SYSEX_END_BYTE) return true;
  midi_SYSEX_buffer[midi_SYSEX_index++] = byte;
  return false;
}

// First byte in the sysex messages
#define SYSEX_MANID 0x7D
// This second byte means its a command for the mcu
#define SYSEX_CMD_BYTE 0x07

/// Queue sysex message
///
/// @param bytes the message, excluding the wrapping `0xF0` and `0xF7`
static void queue_midi_sysex(ByteSpan bytes)
{
  int8_t i = 0;
  const auto enqueue = [&](uint8_t b) {
    if (i == 0) {
      // Packets are 4 bytes, first byte of each packet is 0x4 if start or continuation of sysex
      midi_USB_TX_buffer[midi_USB_TX_index++] = 0x4;
    }
    midi_USB_TX_buffer[midi_USB_TX_index++] = b;
    ++i %= 3;
  };
  enqueue(SYSEX_START_BYTE);
  enqueue(SYSEX_MANID);
  enqueue(SYSEX_CMD_BYTE);
  for (uint8_t b : bytes) enqueue(b);
  enqueue(SYSEX_END_BYTE);
  if (i == 0) {
    // First byte of last packet is 0x5 + number of bytes in packet [1;3]
    midi_USB_TX_buffer[midi_USB_TX_index - 4] = 0x7;
  } else {
    // First byte of last packet is 0x5 + number of bytes in packet [1;3]
    midi_USB_TX_buffer[midi_USB_TX_index - i - 1] = 0x4 + i;
    midi_USB_TX_index += 3 - i;
  }
}

static void handle_midi_sysex(USBD_HandleTypeDef* hUsbDeviceHS)
{
  uint8_t args_len = handleSlaveCommand(&midi_slave_state, midi_SYSEX_buffer[0]);
  if (args_len > 0) {
    assert(args_len <= midi_SYSEX_index);
    handleSlaveCommandArgs(&midi_slave_state, ByteSpan{midi_SYSEX_buffer + 1, args_len});
  }
  if (midi_slave_state.state == READY_TO_RESPOND) {
    queue_midi_sysex(midi_slave_state.tx_buffer);
    midi_slave_state.state = WAITING_FOR_MASTER;
    midi_slave_state.tx_buffer.size = 0;
  }
}

static void handle_midi_message(USBD_HandleTypeDef* hUsbDeviceHS, MIDI_Message message)
{
  if (midi_SYSEX_index > -1) {
    bool res = append_midi_sysex(message.byte0);
    res = res || append_midi_sysex(message.byte1);
    res = res || append_midi_sysex(message.byte2);
    if (res) {
      handle_midi_sysex(hUsbDeviceHS);
      midi_SYSEX_index = -1;
    }
  } else if (message.byte0 == SYSEX_START_BYTE && message.byte1 == SYSEX_MANID && message.byte2 == SYSEX_CMD_BYTE) {
    midi_SYSEX_index = 0;
  } else {
    MIDI_Raspi_TX_queue(message);
    MIDI_USB_TX_queue(message);
  }
}

void MIDI_USB_RX_parse(USBD_HandleTypeDef* hUsbDeviceHS, uint32_t buffer_len_in_bytes)
{
  // each USB midi package is 4 bytes long
  // The first is unused, the three other contain data
  uint16_t messages_size = buffer_len_in_bytes / 4;
  uint8_t* ptr = UserRxBufferFS;

  MIDI_Message message;

  while (messages_size--) {
    ptr++; // discard first byte
    message.byte0 = *ptr++;
    message.byte1 = *ptr++;
    message.byte2 = *ptr++;
    handle_midi_message(hUsbDeviceHS, message);
  }
  MIDI_USB_TX_send(hUsbDeviceHS);
  MIDI_Raspi_TX_send();
}

void MIDI_Raspi_TX_queue(MIDI_Message message)
{
  midi_raspi_TX_buffer[midi_raspi_TX_index++] = message;
}
void MIDI_Raspi_TX_send()
{
  // TODO
  midi_raspi_TX_index = 0;
}

void MIDI_Raspi_RX_onReceive(MIDI_Message message)
{
  MIDI_USB_TX_queue(message);
}
