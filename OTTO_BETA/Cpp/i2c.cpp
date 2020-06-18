#include "i2c.hpp"
#include "gpio.hpp"
#include "instances.hpp"
#include "local_vector.hpp"
#include "span.hpp"

#include <stm32f4xx_hal_i2c.h>

namespace otto::mcu::i2c {

  struct SlaveState {
    enum struct State {
      idle,
      ready_to_respond,
    } state = State::idle;
  } slave_state;

  util::local_vector<std::uint8_t, 32> rx_buffer;
  util::local_vector<std::uint8_t, 32> tx_buffer = {1, 2};

  void error_handler()
  {
    while (true) {
      asm("bkpt 255");
      instances::status_led.toggle();
      HAL_Delay(10);
    }
  }

  void slave_rx_complete(I2C_HandleTypeDef* hi2c)
  {
    if (slave_state.state == SlaveState::State::idle) {
      tx_buffer = {0x55};
      slave_state.state = SlaveState::State::ready_to_respond;
    }
    rx_buffer.clear();
  }

  void slave_tx_complete(I2C_HandleTypeDef* hi2c)
  {
    slave_state.state = SlaveState::State::idle;
    tx_buffer.clear();
  }

  void address_callback(I2C_HandleTypeDef* hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
  {
    if (AddrMatchCode == (command_addr << 1)) {
      if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        rx_buffer.resize(1);
        if (HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, rx_buffer.data(), rx_buffer.size(), I2C_FIRST_FRAME) != HAL_OK) {
          error_handler();
        }
        // } else if (TransferDirection == I2C_DIRECTION_RECEIVE) {
        //   if (HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, tx_buffer.data(), tx_buffer.size(), I2C_FIRST_FRAME) != HAL_OK) {
        //     error_handler();
        //   }
      }
    }
  }

  void init(I2C_HandleTypeDef* hi2c)
  {
    HAL_I2C_RegisterAddrCallback(hi2c, address_callback);
    HAL_I2C_RegisterCallback(hi2c, HAL_I2C_SLAVE_TX_COMPLETE_CB_ID, slave_tx_complete);
    HAL_I2C_RegisterCallback(hi2c, HAL_I2C_SLAVE_RX_COMPLETE_CB_ID, slave_rx_complete);

    instances::main_loop.schedule(0, 1, [hi2c] {
      if (HAL_I2C_GetState(hi2c) == HAL_I2C_STATE_LISTEN && slave_state.state == SlaveState::State::ready_to_respond) {
        if (HAL_I2C_DisableListen_IT(hi2c) != HAL_OK) {
          error_handler();
        }
      }
      if (HAL_I2C_GetState(hi2c) == HAL_I2C_STATE_READY) {
        switch (slave_state.state) {
          case SlaveState::State::idle:
            if (HAL_I2C_EnableListen_IT(hi2c) != HAL_OK) {
              error_handler();
            }
            break;
          case SlaveState::State::ready_to_respond:
            if (HAL_I2C_Slave_Transmit_IT(hi2c, tx_buffer.data(), tx_buffer.size()) != HAL_OK) {
              error_handler();
            }
            break;
          default: break;
        }
      }

      else if (HAL_I2C_GetState(hi2c) == HAL_I2C_STATE_ERROR) {
        switch (HAL_I2C_GetError(hi2c)) {
          case HAL_I2C_ERROR_AF: __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF); break;
          default: break;
        }
        error_handler();
      }
    });
  }

} // namespace otto::mcu::i2c
