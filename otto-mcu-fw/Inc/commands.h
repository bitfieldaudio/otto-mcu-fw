#pragma once

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define XFER_BUFFER_SIZE 32
typedef struct {
  uint8_t data[XFER_BUFFER_SIZE];
  uint8_t size;
} XferBuffer;

typedef struct ByteSpan {
  uint8_t* data;
  uint8_t size;

#ifdef __cplusplus
  ByteSpan(uint8_t* data, uint8_t size) : data(data), size(size) {}
  ByteSpan(XferBuffer& b) : data(b.data), size(b.size) {}

  uint8_t* begin() {
    return data;
  }
  uint8_t* end() {
    return data + size;
  }
#endif
} ByteSpan;

typedef enum {
  READ_INPUTS = 0x00,
  LED_SET = 0x01,
  LEDS_CLEAR = 0x02,
} SLAVE_COMMAND;

typedef enum {
  WAITING_FOR_MASTER,
  COMMAND,
  READY_TO_RESPOND,
} SLAVE_STATE;

typedef struct {
  volatile SLAVE_STATE state;
  volatile SLAVE_COMMAND command;
  XferBuffer tx_buffer;
} SlaveState;

/// Returns the number of bytes to request for args
uint8_t handleSlaveCommand(SlaveState* state, uint8_t command_byte);
void handleSlaveCommandArgs(SlaveState* state, ByteSpan args);

#ifdef __cplusplus
}
#endif
