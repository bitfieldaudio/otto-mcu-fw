#pragma once

#include <atomic>

#include "fixed_size_function.hpp"
#include "gpio.hpp"

namespace otto::mcu {

  struct Encoder {
    GpioPin pin_a;
    GpioPin pin_b;

    Encoder(GpioPin a, GpioPin b) : pin_a(a), pin_b(b) {}

    void init()
    {
      pin_a.init(GpioPin::Mode::it_rising_falling, GpioPin::Pull::up);
      pin_b.init(GpioPin::Mode::input, GpioPin::Pull::up);
    }

    void irq_checked(std::uint16_t pin)
    {
      if (pin != pin_a.pin) return;
      irq();
    }

    void irq()
    {
      bool a_now = pin_a.read();
      bool b_now = pin_b.read();
      if (a_now != a_prev) {
        a_prev = a_now;
        if (a_now == false) {
          value += b_now ? 1 : -1;
        }
      }
    }

    std::int8_t grab_value()
    {
      return value.exchange(0);
    }

    std::atomic<std::int8_t> value = 0;

  private:
    bool a_prev = false;
  };
} // namespace otto::mcu
