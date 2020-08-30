#pragma once

#include "command.hpp"
#include "encoder.hpp"
#include "i2c.hpp"
#include "input.hpp"
#include "keys.hpp"
#include "log.hpp"
#include "scheduler.hpp"
#include "ws2812b.hpp"

namespace otto::mcu::instances {
  extern Scheduler main_loop;
  extern ws2812b::Ws2812bArray leds;
  extern i2c::I2CSlave i2c1;

  extern std::array<Encoder, 4> encoders;

  inline Encoder& blue_encoder = encoders[0];
  inline Encoder& green_encoder = encoders[1];
  inline Encoder& yellow_encoder = encoders[2];
  inline Encoder& red_encoder = encoders[3];

  extern const GpioPin status_led;

  inline void transmit(const Packet& packet)
  {
    log("Transmitting packet");
    i2c1.transmit(packet.to_array());
  }
} // namespace otto::mcu::instances
