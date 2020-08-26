#include <cstdint>
#include <span>
#include <vector>

#include "command.hpp"
#include "instances.hpp"
#include "log.hpp"

namespace otto::mcu {

  void CommandState::handle_args(std::span<uint8_t> args)
  {
    switch (cur_cmd) {
      case Command::led_set: {
        instances::leds[args[0]] = ws2812b::RGBColor::from_bytes(std::span<uint8_t, 3>(args.subspan(1, 3)));
      } break;
      case Command::leds_clear: {
        instances::leds.clear();
      } break;
      case Command::read_inputs: {
        inputs_response();
      } break;
    }
  }

  uint8_t CommandState::handle_cmd(uint8_t cmd_byte)
  {
    if (state == State::idle) {
      cmd_byte = 0;
      state = State::waiting_for_args;
      cur_cmd = static_cast<Command>(cmd_byte);
      switch (cur_cmd) {
        case Command::led_set: return 4;
        case Command::leds_clear: return 0;
        case Command::read_inputs: return 0;
      }
    }
    return 0;
  }

  void CommandState::inputs_response()
  {
    instances::key_table.scan();
    cauto count = instances::key_table.row_count;
    tx_buffer.resize(count + 4);
    for (int r = 0; r < count; r++) {
      cauto& row = instances::key_table.table[r];
      auto& b = tx_buffer[r] = 0;
      for (cauto& cell : row) {
        if (cell) b |= 1;
        b <<= 1;
      }
    }
    for (int i = 0; i < 4; i++) {
      tx_buffer[count + i] = instances::encoders[i].value;
    }
  }

} // namespace otto::mcu
