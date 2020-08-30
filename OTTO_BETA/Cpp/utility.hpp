#pragma once

#include <cstdint>
#include <span>

#define cauto const auto

namespace otto::mcu {

  inline void set_bit(std::uint8_t& byte, std::size_t idx, bool val)
  {
    byte ^= (-(!!val) ^ byte) & (1ul << idx);
  }

  inline void set_bit(std::span<std::uint8_t> array, std::size_t idx, bool val)
  {
    set_bit(array[idx / 8], idx % 8, val);
  }
} // namespace otto::mcu
