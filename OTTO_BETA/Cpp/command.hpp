#pragma once
#include <cstdint>
#include <span>
#include <vector>

#include "instances.hpp"

#define cauto const auto

namespace otto::mcu {

  enum struct Command {
    read_inputs = 0x00,
    led_set = 0x01,
    leds_clear = 0x02,
  };

  struct CommandState {
    enum struct State {
      idle,
      waiting_for_command,
      waiting_for_args,
      received_args,
      transmitting_response,
    } state = State::idle;

    Command cur_cmd;
    std::vector<uint8_t> tx_buffer;

    uint8_t handle_cmd(uint8_t cmd_byte);
    void handle_args(std::span<uint8_t> bytes);
    void inputs_response();
  };
} // namespace otto::mcu
