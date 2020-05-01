#pragma once

#include "otto.h"

namespace otto::mcu::i2c {

  constexpr auto command_addr = 0x77;
  constexpr auto input_addr = 0x70;

  void init();

} // namespace otto::mcu::i2c
