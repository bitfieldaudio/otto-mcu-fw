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
  extern ws2812b::Ws2812bArray leds;
  extern KeyMatrix key_table;

  extern std::array<Encoder, 4> encoders;

  inline Encoder& blue_encoder = encoders[0];
  inline Encoder& green_encoder = encoders[1];
  inline Encoder& yellow_encoder = encoders[2];
  inline Encoder& red_encoder = encoders[3];

  extern const GpioPin status_led;
} // namespace otto::mcu::instances
