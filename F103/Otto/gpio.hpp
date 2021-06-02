#pragma once

#include <cstdint>
#include <utility>

#include "otto.h"
#include <stm32f1xx_hal.h>

namespace otto::mcu {

  template<typename T>
  struct function_ptr_impl;

  template<typename Ret, typename... Args>
  struct function_ptr_impl<Ret(Args...)> {
    using type = Ret(*)(Args...);
  };

  template<typename F>
  using function_ptr = typename function_ptr_impl<F>::type;

  struct GpioPin {
    enum struct Mode : std::uint32_t {
      input = GPIO_MODE_INPUT,
      output_pp = GPIO_MODE_OUTPUT_PP,
      output_od = GPIO_MODE_OUTPUT_OD,
      af_pp = GPIO_MODE_AF_PP,
      af_od = GPIO_MODE_AF_OD,
      analog = GPIO_MODE_ANALOG,
      it_rising = GPIO_MODE_IT_RISING,
      it_falling = GPIO_MODE_IT_FALLING,
      it_rising_falling = GPIO_MODE_IT_RISING_FALLING,
      evt_rising = GPIO_MODE_EVT_RISING,
      evt_falling = GPIO_MODE_EVT_FALLING,
      evt_rising_falling = GPIO_MODE_EVT_RISING_FALLING,
    };

    enum struct Pull : std::uint32_t { none, up, down };

    enum struct Speed : std::uint32_t {
      /// IO works at 2 MHz, please refer to the product datasheet
      freq_low = GPIO_SPEED_FREQ_LOW,
      /// range 12,5 MHz to 50 MHz, please refer to the product datasheet
      freq_medium = GPIO_SPEED_FREQ_MEDIUM,
      /// range 25 MHz to 100 MHz, please refer to the product datasheet
      freq_high = GPIO_SPEED_FREQ_HIGH,
    };

    void init(Mode mode, Pull pull = Pull::none, Speed speed = Speed::freq_low)
    {
      GPIO_InitTypeDef data = {
        .Pin = pin,
        .Mode = static_cast<std::uint32_t>(mode),
        .Pull = static_cast<std::uint32_t>(pull),
        .Speed = static_cast<std::uint32_t>(speed),
      };
      HAL_GPIO_Init(port, &data);
    }

    bool read() const
    {
      auto res = HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET;
      return invert ? !res : res;
    }

    void write(bool state) const
    {
      if (invert) state = !state;
      HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    void toggle() const
    {
      HAL_GPIO_TogglePin(port, pin);
    }

    GPIO_TypeDef* port;
    uint32_t pin;
    bool invert = false;
  };

}

/// Construct a GpioPin struct from the cubemx *_GPIO_Port and *_Pin macros
#define GPIO_PIN(name_prefix, ...) otto::mcu::GpioPin{.port = name_prefix##_GPIO_Port, .pin = name_prefix##_Pin, __VA_ARGS__}
