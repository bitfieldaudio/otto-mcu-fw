#pragma once

#include <array>
#include <bitset>

#include "gpio.hpp"
#include "otto.h"

namespace otto::mcu::keys {

  template<typename Data_, int Rows, int Cols>
  struct KeyMatrix {
    static constexpr int row_count = Rows;
    static constexpr int col_count = Cols;
    using CellData = Data_;
    using States = std::bitset<row_count * col_count>;
    using Row = std::array<CellData, col_count>;
    using Table = std::array<Row, row_count>;

    Table table;
    std::array<GpioPin, row_count> row_pins;
    std::array<GpioPin, col_count> col_pins;
    std::bitset<row_count * col_count> states;

    void init()
    {
      for (auto& pin : col_pins) {
        pin.init(GpioPin::Mode::output_pp);
      }
      for (auto& pin : row_pins) {
        pin.init(GpioPin::Mode::input, GpioPin::Pull::down);
      }
    }

    constexpr static std::uint8_t idx_of(std::uint8_t r, std::uint8_t c)
    {
      return c * row_count + r;
    }

    /// Update the table
    bool scan()
    {
      bool has_changed = false;
      for (int c = 0; c < col_count; c++) {
        col_pins[c].write(true);
        for (int r = 0; r < row_count; r++) {
          bool cur = row_pins[r].read();
          has_changed = has_changed || states[idx_of(r, c)] != cur;
          states[idx_of(r, c)] = cur;
        }
        col_pins[c].write(false);
      }
      return has_changed;
    }
  };
} // namespace otto::mcu::keys
