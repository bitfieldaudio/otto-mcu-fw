#pragma once

#include <array>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"

#include "otto.hpp"

#include "algorithm.hpp"

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
      return {static_cast<uint8_t>(r * f), static_cast<uint8_t>(g * f), static_cast<uint8_t>(b * f)};
    }

    bool operator==(RGBColor const& rhs)
    {
      return r == rhs.r && g == rhs.g && b == rhs.b;
    }
  };

  struct Ws2812bRef {
    Ws2812bRef(std::uint8_t* data_ptr, bool& needs_update) : data_ptr_(data_ptr), needs_update_(needs_update) {}

    Ws2812bRef& operator=(RGBColor col)
    {
      if (col == *this) return *this;
      auto buf_iter = data_ptr_;
      buf_iter = util::copy(ws2812b_mapping[col.r], buf_iter);
      buf_iter = util::copy(ws2812b_mapping[col.g], buf_iter);
      buf_iter = util::copy(ws2812b_mapping[col.b], buf_iter);
      needs_update_ = true;
      return *this;
    }

    operator RGBColor()
    {
      return {
        .r = byte_from_mapping({data_ptr_[0 + 0], data_ptr_[0 + 1], data_ptr_[0 + 2], data_ptr_[0 + 3]}),
        .g = byte_from_mapping({data_ptr_[4 + 0], data_ptr_[4 + 1], data_ptr_[4 + 2], data_ptr_[4 + 3]}),
        .b = byte_from_mapping({data_ptr_[8 + 0], data_ptr_[8 + 1], data_ptr_[8 + 2], data_ptr_[8 + 3]}),
      };
    }

  private:
    std::uint8_t* const data_ptr_;
    bool& needs_update_;
  };

  struct Ws2812bIter {
    Ws2812bIter(std::uint8_t* ptr, bool* needs_update) : ptr_(ptr), needs_update_(needs_update) {}

    Ws2812bRef operator*()
    {
      return {ptr_, *needs_update_};
    }

    Ws2812bIter& operator++()
    {
      ptr_ += 12;
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
    std::uint8_t* ptr_ = nullptr;
    bool* needs_update_ = nullptr;
  };

  template<int N>
  struct Ws2812bArray {
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
      util::fill(spi_buf_, ws2812b_mapping[0][0]);
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
      return {spi_buf_.begin(), &needs_update_};
    }

    iterator end()
    {
      return {spi_buf_.end(), &needs_update_};
    }

    Ws2812bRef operator[](int idx)
    {
      return {spi_buf_.data() + idx * 12, needs_update_};
    }

  private:
    void transmit()
    {
      auto res = HAL_SPI_Transmit_DMA(&hspi_, spi_buf_.data(), spi_buf_.size() + zeros_.size());
      while (__HAL_SPI_GET_FLAG(&hspi_, SPI_FLAG_BSY))
        ;
      if (res != HAL_OK) {
        printf("ERROR");
      }
    }

    SPI_HandleTypeDef& hspi_;
    bool needs_update_;
    std::array<std::uint8_t, size()* 3 * 4> spi_buf_ = {0};
    std::array<std::uint8_t, 2> zeros_ = {0};
  };


  constexpr std::array colors = {
    RGBColor{0x80, 0x80, 0x80}, RGBColor{0x80, 0x80, 0x00}, RGBColor{0x80, 0x00, 0x00}, RGBColor{0x80, 0x00, 0x80},
    RGBColor{0x00, 0x00, 0x80}, RGBColor{0x00, 0x80, 0x80}, RGBColor{0x00, 0x80, 0x00},
  };

  template<int N>
  void led_cascade_colors(Ws2812bArray<N>& leds)
  {
    main_loop.schedule_cond_repeat([&leds, c = 0U, l = 0] () mutable {
      leds[l] = colors[c] * 0.5;
      l++;
      if (l == leds.size()) {
        c++;
        l = 0;
      }
      return c == colors.size() ? 0 : 50;
    });
  }

  template<int N>
  void led_pulse_colors(Ws2812bArray<N>& leds)
  {
    for (auto color : colors) {
      for (float f = 0; f <= 1; f += 0.01) {
        auto fcolor = color * f;
        for (int i = 0; i < leds.size(); i++) {
          leds[i] = fcolor;
        }
        leds.maybe_update();
        HAL_Delay(10);
      }
      for (float f = 1; f >= 0; f -= 0.01) {
        auto fcolor = color * f;
        for (int i = 0; i < leds.size(); i++) {
          leds[i] = fcolor;
        }
        leds.maybe_update();
        HAL_Delay(10);
      }
    }
  }

} // namespace otto::mcu::ws2812b
