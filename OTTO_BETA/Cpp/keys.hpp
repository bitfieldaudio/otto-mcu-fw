#pragma once

#include <array>

#include "otto.h"
#include "gpio.hpp"

namespace otto::mcu::keys {
  template<typename Data_, int Rows, int Cols>
  struct KeyMatrix {
    static constexpr int row_count = Rows;
    static constexpr int col_count = Cols;
    using CellData = Data_;
    struct Cell {
      Cell(CellData data = {}) : data(data){};
      CellData data;
      bool is_down = false;
    };
    using Row = std::array<Cell, col_count>;
    using Table = std::array<Row, row_count>;

    Table table;
    std::array<GpioPin, row_count> row_pins;
    std::array<GpioPin, col_count> col_pins;

    void init()
    {
      for (auto& pin : col_pins) {
        pin.init(GpioPin::Mode::output_pp);
      }
      for (auto& pin : row_pins) {
        pin.init(GpioPin::Mode::input, GpioPin::Pull::down);
      }
    }

    /// Update the table
    void scan()
    {
      for (int c = 0; c < col_count; c++) {
        col_pins[c].write(true);
        for (int r = 0; r < row_count; r++) {
          bool cur = row_pins[r].read();
          bool& prev = table[r][c].is_down;
          prev = cur;
        }
        col_pins[c].write(false);
      }
    }
  };
} // namespace otto::mcu::keys
