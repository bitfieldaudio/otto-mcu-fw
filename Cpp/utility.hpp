#pragma once

#include <cstdint>
#include <span>

#define cauto const auto
#define FWD(X) std::forward<decltype(X)>(X)

namespace otto::mcu {

  inline void set_bit(std::uint8_t& byte, std::size_t idx, bool val)
  {
    byte ^= (-(!!val) ^ byte) & (1ul << idx);
  }

  inline void set_bit(std::span<std::uint8_t> array, std::size_t idx, bool val)
  {
    set_bit(array[idx / 8], idx % 8, val);
  }

  inline bool get_bit(std::uint8_t byte, std::size_t idx)
  {
    return (byte & (1ul << idx)) != 0;
  }

  inline bool get_bit(std::span<const std::uint8_t> array, std::size_t idx)
  {
    return get_bit(array[idx / 8], idx % 8);
  }
} // namespace otto::mcu
