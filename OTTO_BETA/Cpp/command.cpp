#include <cstdint>
#include <span>
#include <vector>

namespace otto::mcu {

  enum struct Command {
    read_inputs = 0x00,
    led_set = 0x01,
    leds_clear = 0x02,
  };

  struct CommandState {
    enum struct State { idle, waiting_for_args, ready_to_respond } state;

    Command cur_cmd;
    std::vector<uint8_t> tx_buffer;

    uint8_t handle_cmd(uint8_t cmd_byte);
    void handle_args(std::span<uint8_t> bytes) {}
  };

  uint8_t CommandState::handle_cmd(uint8_t cmd_byte)
  {
    if (state == State::idle) {
      cur_cmd = static_cast<Command>(cmd_byte);
      switch (cur_cmd) {
        case Command::led_set: state = State::waiting_for_args; return 5;
        case Command::leds_clear: state = State::waiting_for_args; return 1;
        case Command::read_inputs:  return 0;
      }
    }
    return 0;
  }

} // namespace otto::mcu
