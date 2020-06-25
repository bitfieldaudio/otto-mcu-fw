#include "i2c.hpp"
#include "command.hpp"
#include "gpio.hpp"
#include "instances.hpp"
#include "local_vector.hpp"
#include "log.hpp"

#include <stm32f4xx_hal_i2c.h>

namespace otto::mcu::i2c {

  namespace {
    CommandState slave_state;

    util::local_vector<std::uint8_t, 32> rx_buffer;
  } // namespace

  void error_handler()
  {
    log("I2C Error handler");
    //while (true) {
    //  asm("bkpt 255");
    //  instances::status_led.toggle();
    //  HAL_Delay(10);
    //}
  }

  void slave_rx_complete(I2C_HandleTypeDef* hi2c)
  {
    log("Slave received %d bytes", rx_buffer.size());
    if (slave_state.state == CommandState::State::idle) {
      slave_state.handle_cmd(rx_buffer[0]);
    }
    rx_buffer.clear();
  }

  void slave_tx_complete(I2C_HandleTypeDef* hi2c)
  {
    log("Slave transmitted %d bytes", slave_state.tx_buffer.size());
    slave_state.state = CommandState::State::idle;
    slave_state.tx_buffer.clear();
  }

  void error_callback(I2C_HandleTypeDef* hi2c) {
    log("I2C Error: %d", hi2c->ErrorCode);
    slave_state.state = CommandState::State::idle;
  }

  void address_callback(I2C_HandleTypeDef* hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
  {
    if (AddrMatchCode == (command_addr << 1)) {
      if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        log("Address callback, transmit");
        rx_buffer.resize(1);
        log("AC, receiving %d byte", rx_buffer.size());
        if (HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, rx_buffer.data(), rx_buffer.size(), I2C_FIRST_FRAME) != HAL_OK) {
          error_handler();
        }
      }
      if (TransferDirection == I2C_DIRECTION_RECEIVE) {
        log("Address callback, receive");
        if (slave_state.tx_buffer.size() > 0) {
          log("AC, transmitting %d byte", slave_state.tx_buffer.size());
          //if (auto status = HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, slave_state.tx_buffer.data(),
          //                                                slave_state.tx_buffer.size(), I2C_FIRST_FRAME);
          //    status != HAL_OK) {
          //  log("Error transmitting: %d", status);
          //  error_handler();
          //}
        } else {
          log("No data to transmit");
        }
      }
    }
  }

  void init(I2C_HandleTypeDef* hi2c)
  {
    log("Initializing i2c");
    HAL_I2C_RegisterAddrCallback(hi2c, address_callback);
    HAL_I2C_RegisterCallback(hi2c, HAL_I2C_SLAVE_TX_COMPLETE_CB_ID, slave_tx_complete);
    HAL_I2C_RegisterCallback(hi2c, HAL_I2C_SLAVE_RX_COMPLETE_CB_ID, slave_rx_complete);
    HAL_I2C_RegisterCallback(hi2c, HAL_I2C_ERROR_CB_ID, error_callback);

    instances::main_loop.schedule(0, 1, [hi2c] {
      if (HAL_I2C_GetState(hi2c) == HAL_I2C_STATE_LISTEN &&
          slave_state.state == CommandState::State::ready_to_respond) {
        log("Disable listen");
        if (HAL_I2C_DisableListen_IT(hi2c) != HAL_OK) {
          error_handler();
        }
      }
      if (HAL_I2C_GetState(hi2c) == HAL_I2C_STATE_READY) {
        switch (slave_state.state) {
          case CommandState::State::idle:
            log("Enable listen");
            if (HAL_I2C_EnableListen_IT(hi2c) != HAL_OK) {
              error_handler();
            }
            break;
          case CommandState::State::ready_to_respond:
             log("Transmitting %d bytes", slave_state.tx_buffer.size());
             if (HAL_I2C_Slave_Transmit_IT(hi2c, slave_state.tx_buffer.data(), slave_state.tx_buffer.size()) !=
               HAL_OK) {
                error_handler();
              }
             slave_state.state = CommandState::State::idle;
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
