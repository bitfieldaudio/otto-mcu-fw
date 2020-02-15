#include <cassert>

#include <commands.h>
#include "encoders.h"
#include "keys.h"
#include "ws2812b.h"

void generateInputsResponse(XferBuffer& buffer)
{
  fillKeymap();
  for (uint8_t i = 0; i < NUM_ROWS; i++) {
    buffer.data[i] = gKeymap[i];
  }
  for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
    buffer.data[NUM_ROWS + i] = gEncoders[i].value;
  }
  buffer.size = NUM_ROWS + NUM_ENCODERS;
}

uint8_t handleSlaveCommand(SlaveState* state, uint8_t command_byte)
{
  if (state->state == WAITING_FOR_MASTER) {
    switch (command_byte) {
      case LED_SET: // Set LED Command
        state->command = LED_SET;
        state->state = COMMAND;
        return 5;
      case LEDS_CLEAR: // Clear LED Command
        state->command = LEDS_CLEAR;
        state->state = COMMAND;
        return 1;
      case READ_INPUTS: [[fallthrough]]; // Read Inputs Command
      default:
        generateInputsResponse(state->tx_buffer);
        state->command = READ_INPUTS;
        state->state = READY_TO_RESPOND;
        return 0;
    }
  }
  return 0;
}

void handleSlaveCommandArgs(SlaveState* state, ByteSpan args)
{
  if (state->state == COMMAND) {
    switch (state->command) {
      case LED_SET:
        assert(args.size == 5);
        // gRxBuffer[0] is the LED string number
        // gRxBuffer[1] is the LED number in that string
        // gRxBuffer[2] is the Red Byte
        // gRxBuffer[3] is the Green Byte
        // gRxBuffer[4] is the Blue Byte
        setPixelColor(gLED_STRINGS[args.data[0]], args.data[1], args.data[2], args.data[3], args.data[4]);
        generateInputsResponse(state->tx_buffer);
        state->state = READY_TO_RESPOND;
        break;
      case LEDS_CLEAR:
        clearPixels(gLED_STRINGS[args.data[0]]);
        generateInputsResponse(state->tx_buffer);
        state->state = READY_TO_RESPOND;
        break;
      default: break;
    }
  }
}
