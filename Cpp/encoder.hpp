#pragma once

#include <atomic>

#include "algorithm.hpp"
#include "fixed_size_function.hpp"
#include "gpio.hpp"

namespace otto::mcu {

  struct Encoder {
    GpioPin pin_a;
    GpioPin pin_b;

    static constexpr std::array<std::uint8_t, 4> cw_states = {
      0b0100, 0b0010, 0b1011, 0b1101
    };
    static constexpr std::array<std::uint8_t, 4> ccw_states = {
      0b1110, 0b1000, 0b0001, 0b0111
    };

    Encoder(GpioPin a, GpioPin b) : pin_a(a), pin_b(b) {}

    void init()
    {
      pin_a.init(GpioPin::Mode::input, GpioPin::Pull::up);
      pin_b.init(GpioPin::Mode::input, GpioPin::Pull::up);
    }

    void poll()
    {
      // LP Filter
      state_ = (state_ << 1) | pin_a.read();
      state_ = (state_ << 1) | pin_b.read();
      state_ &= 0b1111;
      if (util::contains(cw_states, state_)) {
        value++;
      } else if (util::contains(ccw_states, state_)) {
        value--;
      }
    }

    std::int8_t grab_value()
    {
      return value.exchange(0);
    }

    std::atomic<std::int8_t> value = 0;

  private:
    std::uint16_t state_ = 0;
  };
} // namespace otto::mcu
