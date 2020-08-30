#pragma once
#include <cstdint>
#include <span>
#include <vector>
#include <algorithm>

namespace otto::mcu {

  enum struct Command {
    none = 0,
    led_set = 1,
    leds_clear = 2,
    key_events = 3,
    encoder_events = 4,
  };

  struct Packet {
    Command cmd = Command::none;
    std::array<std::uint8_t, 16> data = {0};

    std::array<std::uint8_t, 17> to_array() const
    {
      std::array<std::uint8_t, 17> res;
      res[0] = static_cast<std::uint8_t>(cmd);
      std::ranges::copy(data, res.begin() + 1);
      return res;
    }
  };

} // namespace otto::mcu
