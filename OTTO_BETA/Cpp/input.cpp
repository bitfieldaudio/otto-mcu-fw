#include "input.hpp"
#include "command.hpp"
#include "utility.hpp"

#include "instances.hpp"

namespace otto::mcu {

  void InputManager::poll()
  {
    if (!matrix_.scan()) return;
    Packet packet = {Command::key_events};
    std::span<std::uint8_t> presses = {packet.data.data(), 8};
    std::span<std::uint8_t> releases = {packet.data.data() + 8, 8};
    for (int c = 0; c < matrix_.col_count; c++) {
      for (int r = 0; r < matrix_.row_count; r++) {
        cauto idx = matrix_.idx_of(r, c);
        if (old_states_[idx] != matrix_.states[idx]) {
          set_bit(matrix_.states[idx] ? presses : releases, static_cast<std::uint8_t>(matrix_.table[r][c]), true);
        }
      }
    }

    instances::transmit(packet);
    old_states_ = matrix_.states;
  }
  void InputManager::init()
  {
    matrix_.init();
  }
} // namespace otto::mcu
