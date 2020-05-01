#include "i2c.hpp"
#include "gpio.hpp"
#include "local_vector.hpp"
#include "span.hpp"

namespace otto::mcu::i2c {

  struct I2CSlaveState {
  } slave_state;

  util::local_vector<std::uint8_t, 32> rx_buffer;
  util::local_vector<std::uint8_t, 32> tx_buffer;

  void slave_rx_complete(I2C_HandleTypeDef* hi2c)
  {
    rx_buffer.resize(1);
    if (HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, rx_buffer.data(), rx_buffer.size(), I2C_LAST_FRAME) != HAL_OK) {
      /* Transfer error in reception process */
      status_led.toggle();
      Error_Handler();
    }
  }
  void slave_tx_complete(I2C_HandleTypeDef* hi2c) {

  }
  void address_callback(I2C_HandleTypeDef* hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
  {
    if (AddrMatchCode == input_addr) {
      if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        rx_buffer.resize(1);
        if (HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, rx_buffer.data(), rx_buffer.size(), I2C_FIRST_FRAME) != HAL_OK) {
          Error_Handler();
        }
      } else if (TransferDirection == I2C_DIRECTION_RECEIVE) {
        if (HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, tx_buffer.data(), tx_buffer.size(),
                                          I2C_FIRST_FRAME) != HAL_OK) {
          Error_Handler();
        }
      }
    }
  }

  void init(I2C_HandleTypeDef* hi2c)
  {
    HAL_I2C_RegisterAddrCallback(hi2c, address_callback);
    HAL_I2C_RegisterCallback(hi2c, HAL_I2C_SLAVE_TX_COMPLETE_CB_ID, slave_tx_complete);
    HAL_I2C_RegisterCallback(hi2c, HAL_I2C_SLAVE_RX_COMPLETE_CB_ID, slave_rx_complete);
  }
} // namespace otto::mcu::i2c
