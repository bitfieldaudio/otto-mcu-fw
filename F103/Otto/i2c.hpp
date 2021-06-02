#pragma once

#include <stm32f1xx.h>
#include <span>
#include "fixed_size_function.hpp"
#include "local_vector.hpp"

namespace otto::mcu::i2c {

  constexpr auto command_addr = 0x77;

  enum struct State {
    waiting,
    transmitting,
    receiving,
    received_data_ready,
  };

  template<std::regular T, std::size_t Size>
  struct Ringbuf {
    using value_type = T;

    void push_back(const value_type& vt)
    {
      data_[tail_ % Size] = vt;
      tail_++;
    }

    value_type* peek_front()
    {
      if (head_ == tail_) return nullptr;
      return data_.data() + (head_ % Size);
    }

    value_type* pop_front()
    {
      auto res = peek_front();
      if (res != nullptr) head_++;
      return res;
    }

  private:
    std::array<value_type, Size> data_;
    std::size_t head_ = 0;
    std::size_t tail_ = 0;
  };

  struct I2CSlave {
    using PacketData = std::array<std::uint8_t, 17>;
    State state = State::waiting;

    void init();
    void deinit();

    void irq_handle_ev();
    void irq_handle_er();

    void transmit(PacketData);

    fixed_size_function<void(PacketData data), 32, construct_type::move> rx_callback = nullptr;

    void poll();
  private:

    I2C_TypeDef& regs = *I2C1;
    Ringbuf<PacketData, 16> tx_buffer;
    PacketData current_tx = {0};
    unsigned tx_idx = 0;
    util::local_vector<std::uint8_t, 128> rx_buffer;
  };

} // namespace otto::mcu::i2c

/// MIDI over I2C RPI/MCU communication protocol:
///
/// RPI as master
///
/// Message from master ->
