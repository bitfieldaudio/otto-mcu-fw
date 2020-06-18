#pragma once

#include "encoder.hpp"
#include "keys.hpp"
#include "scheduler.hpp"
#include "ws2812b.hpp"

namespace otto::mcu::instances {
  struct KeyData {
    int led_idx = -1;
  };
  using KeyMatrix = keys::KeyMatrix<KeyData, 8, 8>;

  extern Scheduler main_loop;
  extern ws2812b::Ws2812bArray<54> leds;
  extern KeyMatrix key_table;

  extern Encoder blue_encoder;
  extern Encoder green_encoder;
  extern Encoder yellow_encoder;
  extern Encoder red_encoder;

  extern const GpioPin status_led;
} // namespace otto::mcu::instances
