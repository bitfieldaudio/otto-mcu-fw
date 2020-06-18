#pragma once

#include <array>
#include "scheduler.hpp"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"

#include "algorithm.hpp"

namespace otto::mcu::instances {
  extern Scheduler main_loop;
}

namespace otto::mcu::ws2812b {

  using FourBytes = std::array<std::uint8_t, 4>;

  constexpr std::array<std::uint8_t, 2> ws2812b_bits = {0b0100, 0b0111};

  constexpr std::array ws2812b_mapping = [] {
    std::array<FourBytes, 256> res = {};
    for (int i = 0; i < 256; i++) {
      auto& row = res[i];
      for (int j = 0; j < 4; j++) {
        row[j] = [&] {
          switch ((i >> 2 * (3 - j)) & 0b11) {
            case 0b00: return (ws2812b_bits[0] << 4) | ws2812b_bits[0];
            case 0b01: return (ws2812b_bits[0] << 4) | ws2812b_bits[1];
            case 0b10: return (ws2812b_bits[1] << 4) | ws2812b_bits[0];
            case 0b11: return (ws2812b_bits[1] << 4) | ws2812b_bits[1];
          }
          __builtin_unreachable();
        }();
      }
    }
    return res;
  }();

  constexpr std::uint8_t byte_from_mapping(FourBytes bytes)
  {
    std::uint8_t res = 0;
    for (int i = 0; i < 4; i++) {
      auto b = [&] {
        switch (bytes[i]) {
          case (ws2812b_bits[0] << 4) | ws2812b_bits[0]: return (std::uint8_t) 0b00;
          case (ws2812b_bits[0] << 4) | ws2812b_bits[1]: return (std::uint8_t) 0b01;
          case (ws2812b_bits[1] << 4) | ws2812b_bits[0]: return (std::uint8_t) 0b10;
          case (ws2812b_bits[1] << 4) | ws2812b_bits[1]: return (std::uint8_t) 0b11;
        };
        __builtin_unreachable();
      }();
      res = res | (b << 2 * (3 - i));
    }
    return res;
  }

  struct RGBColor {
    std::uint8_t r = 0, g = 0, b = 0;

    RGBColor operator*(float f) const
    {
      return {static_cast<std::uint8_t>(r * f), static_cast<std::uint8_t>(g * f), static_cast<std::uint8_t>(b * f)};
    }

    RGBColor operator/(std::uint8_t v) const
    {
      return {static_cast<std::uint8_t>(r / v), static_cast<std::uint8_t>(g / v), static_cast<std::uint8_t>(b / v)};
    }

    bool operator==(RGBColor const& rhs)
    {
      return r == rhs.r && g == rhs.g && b == rhs.b;
    }
  };

  struct Ws2812bRef {
    Ws2812bRef(RGBColor* data_ptr, bool& needs_update) : data_ptr_(data_ptr), needs_update_(needs_update) {}

    Ws2812bRef& operator=(RGBColor col)
    {
      if (col == *this) return *this;
      *data_ptr_ = col;
      needs_update_ = true;
      return *this;
    }

    operator RGBColor()
    {
      return *data_ptr_;
    }

  private:
    RGBColor* const data_ptr_;
    bool& needs_update_;
  };

  struct Ws2812bIter {
    Ws2812bIter(RGBColor* ptr, bool* needs_update) : ptr_(ptr), needs_update_(needs_update) {}

    Ws2812bRef operator*()
    {
      return {ptr_, *needs_update_};
    }

    Ws2812bIter& operator++()
    {
      ptr_++;
      return *this;
    }

    Ws2812bIter operator++(int)
    {
      auto old = *this;
      ++(*this);
      return old;
    }

    bool operator==(const Ws2812bIter& rhs) const
    {
      return ptr_ == rhs.ptr_;
    }

    bool operator!=(const Ws2812bIter& rhs) const
    {
      return ptr_ != rhs.ptr_;
    }

  private:
    RGBColor* ptr_ = nullptr;
    bool* needs_update_ = nullptr;
  };

  template<int N>
  struct Ws2812bArray {
    static constexpr int max_colors_on = 16;
    static constexpr int div_colors_by = 2;

    using iterator = Ws2812bIter;

    Ws2812bArray(SPI_HandleTypeDef& hspi) : hspi_(hspi)
    {
      clear();
    }

    void init()
    {
      HAL_SPI_Transmit_DMA(&hspi_, zeros_.data(), zeros_.size());
      while (__HAL_SPI_GET_FLAG(&hspi_, SPI_FLAG_BSY))
        ;
    }

    void clear()
    {
      util::fill(colors_, RGBColor{0, 0, 0});
      needs_update_ = true;
    }

    void maybe_update()
    {
      if (needs_update_) send_update();
    }
    void send_update()
    {
      transmit();
      needs_update_ = false;
    }

    constexpr static int size()
    {
      return N;
    }

    iterator begin()
    {
      return {colors_.begin(), &needs_update_};
    }

    iterator end()
    {
      return {colors_.end(), &needs_update_};
    }

    Ws2812bRef operator[](int idx)
    {
      return {colors_.data() + idx, needs_update_};
    }

  private:
    void transmit()
    {
      fill_spi_buf();
      while (__HAL_SPI_GET_FLAG(&hspi_, SPI_FLAG_BSY))
        ;
      auto res = HAL_SPI_Transmit_DMA(&hspi_, spi_buf_.data(), spi_buf_.size());
      if (res != HAL_OK) {
        printf("ERROR");
      }
    }

    void fill_spi_buf()
    {
      int leds_sum = 0;
      constexpr auto max_sum = max_colors_on * 256;
      auto buf_iter = spi_buf_.begin();
      for (auto color : colors_) {
        color = color / div_colors_by;
        leds_sum += color.r + color.g + color.b;
        if (leds_sum >= max_sum) color = {0, 0, 0};
        buf_iter = util::copy(ws2812b_mapping[color.r], buf_iter);
        buf_iter = util::copy(ws2812b_mapping[color.g], buf_iter);
        buf_iter = util::copy(ws2812b_mapping[color.b], buf_iter);
      }
    }

    SPI_HandleTypeDef& hspi_;
    bool needs_update_;
    std::array<RGBColor, size()> colors_ = {0};
    std::array<std::uint8_t, size()* 3 * 4> spi_buf_ = {0};
    std::array<std::uint8_t, 2> zeros_ = {0};
  };


  constexpr std::array colors = {
    RGBColor{0xFF, 0xFF, 0xFF}, RGBColor{0xFF, 0xFF, 0x00}, RGBColor{0xFF, 0x00, 0x00}, RGBColor{0xFF, 0x00, 0xFF},
    RGBColor{0x00, 0x00, 0xFF}, RGBColor{0x00, 0xFF, 0xFF}, RGBColor{0x00, 0xFF, 0x00},
  };

  template<int N>
  Task led_cascade_colors(Ws2812bArray<N>& leds)
  {
    for (auto&& c : colors) {
      for (auto&& l : leds) {
        l = c;
        co_await instances::main_loop.suspend_for(50);
      }
    }
  }

  template<int N>
  Task led_pulse_colors(Ws2812bArray<N>& leds)
  {
    for (auto color : colors) {
      for (float f = 0; f <= 1; f += 0.01) {
        auto fcolor = color * f;
        for (int i = 0; i < leds.size(); i++) {
          leds[i] = fcolor;
        }
        leds.maybe_update();
        co_await instances::main_loop.suspend_for(10);
      }
      for (float f = 1; f >= 0; f -= 0.01) {
        auto fcolor = color * f;
        for (int i = 0; i < leds.size(); i++) {
          leds[i] = fcolor;
        }
        leds.maybe_update();
        co_await instances::main_loop.suspend_for(10);
      }
    }
  }

  template<int N>
  void led_test_zeros(Ws2812bArray<N>& leds)
  {
    while (true) {
      leds[0] = RGBColor{0, 0, 0};
      leds.send_update();
      co_await instances::main_loop.suspend_for(1000);
    }
  }

} // namespace otto::mcu::ws2812b
