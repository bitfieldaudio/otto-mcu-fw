#include <cstdint>
#include <span>
#include <vector>

#include "command.hpp"
#include "instances.hpp"

namespace otto::mcu {

  void CommandState::handle_args(std::span<uint8_t> bytes)
  {
    state = State::idle;
  }

  uint8_t CommandState::handle_cmd(uint8_t cmd_byte)
  {
    if (state == State::idle) {
      cur_cmd = static_cast<Command>(cmd_byte);
      inputs_response();
      state = State::ready_to_respond;
      return 0;
      switch (cur_cmd) {
        case Command::led_set: state = State::waiting_for_args; return 5;
        case Command::leds_clear: state = State::waiting_for_args; return 1;
        case Command::read_inputs: inputs_response(); state = State::ready_to_respond;
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
