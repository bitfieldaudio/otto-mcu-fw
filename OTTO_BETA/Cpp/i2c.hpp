#pragma once

#include "otto.h"

#include "boost_sml.hpp"

namespace otto::mcu::i2c {

  constexpr auto command_addr = 0x77;
  constexpr auto input_addr = 0x70;

  void init(I2C_HandleTypeDef*);

} // namespace otto::mcu::i2c

/// MIDI over I2C RPI/MCU communication protocol:
/// 
/// RPI as master
/// 
/// Message from master -> 
