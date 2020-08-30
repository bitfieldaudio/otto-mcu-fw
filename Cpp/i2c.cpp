#include "i2c.hpp"
#include "command.hpp"
#include "gpio.hpp"
#include "instances.hpp"
#include "local_vector.hpp"
#include "log.hpp"
#include "utility.hpp"

#include <stm32f4xx_hal_i2c.h>

// Redefine to remove deprecated compound assignment to volatile
#undef SET_BIT
#undef CLEAR_BIT
#define SET_BIT(REG, BIT) (REG) = (REG) | (BIT)
#define CLEAR_BIT(REG, BIT) (REG) = (REG) & ~(BIT)

namespace otto::mcu::i2c {

  void I2CSlave::init()
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    // I2C1 GPIO Configuration
    // PB8 = SCL, PB9 = SDA
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);

    CLEAR_BIT(regs.CR1, I2C_CR1_PE);

    SET_BIT(regs.CR1, I2C_CR1_SWRST);
    CLEAR_BIT(regs.CR1, I2C_CR1_SWRST);

    cauto pclk1 = HAL_RCC_GetPCLK1Freq();
    cauto freqrange = pclk1 / 1'000'000;

    MODIFY_REG(regs.CR2, I2C_CR2_FREQ, freqrange);

    cauto rise_time = (freqrange) + 1U;

    MODIFY_REG(regs.TRISE, I2C_TRISE_TRISE, rise_time);

    cauto speed = I2C_SPEED_STANDARD(pclk1, 100000);
    MODIFY_REG(regs.CCR, (I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR), speed);

    MODIFY_REG(regs.CR1, (I2C_CR1_ENGC | I2C_CR1_NOSTRETCH), (I2C_GENERALCALL_DISABLED | I2C_NOSTRETCH_ENABLED));
    MODIFY_REG(regs.OAR1, (I2C_OAR1_ADDMODE | I2C_OAR1_ADD8_9 | I2C_OAR1_ADD1_7 | I2C_OAR1_ADD0),
               (I2C_ADDRESSINGMODE_7BIT | (0x77 << 1)));
    MODIFY_REG(regs.OAR2, (I2C_OAR2_ENDUAL | I2C_OAR2_ADD2), (I2C_DUALADDRESS_DISABLED));

    SET_BIT(regs.CR1, I2C_CR1_PE);

    CLEAR_BIT(regs.CR1, I2C_CR1_POS);
    SET_BIT(regs.CR1, I2C_CR1_ACK);

    SET_BIT(regs.CR2, I2C_CR2_ITBUFEN | I2C_CR2_ITERREN | I2C_CR2_ITEVTEN);
  }

  void I2CSlave::deinit()
  {
    // Disable clock
    CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  }

  void I2CSlave::irq_handle_ev()
  {
    cauto sr1 = regs.SR1;
    cauto addr = READ_BIT(sr1, I2C_SR1_ADDR);
    if (addr) {
      // ADDR is cleared by reading sr2 after reading sr1
      cauto sr2 = regs.SR2;
      if (READ_BIT(sr2, I2C_SR2_TRA)) {
        state = State::transmitting;
        if (auto* p = tx_buffer.peek_front(); p != nullptr) {
          current_tx = *tx_buffer.pop_front();
        } else {
          current_tx = {0};
        }
        tx_idx = 0;
      } else {
        state = State::receiving;
      }
    } else if (state == State::transmitting && READ_BIT(sr1, I2C_SR1_TXE)) {
      // Datasheet fig 241: EV3
      // Transmit the next byte
      regs.DR = tx_idx < current_tx.size() ? current_tx[tx_idx] : 0;
      tx_idx++;
    } else if (state == State::receiving && READ_BIT(sr1, I2C_SR1_RXNE)) {
      rx_buffer.push_back(regs.DR);
    } else if (state == State::receiving && READ_BIT(sr1, I2C_SR1_STOPF)) {
      // Clear STOPF by writing to CR1
      regs.CR1 = regs.CR1;
      // Disable ack until data has been read
      CLEAR_BIT(regs.CR1, I2C_CR1_ACK);
      state = State::received_data_ready;
    }
  }

  void I2CSlave::irq_handle_er()
  {
    cauto sr1 = regs.SR1;
    if (state == State::transmitting && READ_BIT(sr1, I2C_SR1_AF)) {
      // Datasheet fig 241: EV3-2
      // Was transmitting, got nack / stop condition.
      // If the whole packet was transmitted, pop it off
      state = State::waiting;
      CLEAR_BIT(regs.SR1, I2C_SR1_AF);
    } else {
      otto::mcu::log("ER %s%s%s%s%s%s%s",                               //
                     READ_BIT(sr1, I2C_SR1_BERR) ? "BERR " : "",        //
                     READ_BIT(sr1, I2C_SR1_ARLO) ? "ARLO " : "",        //
                     READ_BIT(sr1, I2C_SR1_AF) ? "AF " : "",            //
                     READ_BIT(sr1, I2C_SR1_OVR) ? "OVR " : "",          //
                     READ_BIT(sr1, I2C_SR1_PECERR) ? "PECERR " : "",    //
                     READ_BIT(sr1, I2C_SR1_TIMEOUT) ? "TIMEOUT " : "",  //
                     READ_BIT(sr1, I2C_SR1_SMBALERT) ? "SMBALERT " : "" //
      );
      SET_BIT(regs.CR1, I2C_CR1_ACK);
      regs.SR1 = 0;
      tx_idx = 0;
      state = State::waiting;
    }
  }

  void I2CSlave::poll()
  {
    if (state == State::received_data_ready) {
      if (rx_buffer.size() >= 17) {
        PacketData p;
        std::ranges::copy(rx_buffer, p.begin());
        if (rx_callback) rx_callback(p);
      }
      rx_buffer.clear();
      state = State::waiting;
      SET_BIT(regs.CR1, I2C_CR1_ACK);
    }
  }

  void I2CSlave::transmit(PacketData d)
  {
    tx_buffer.push_back(d);
  }
} // namespace otto::mcu::i2c

extern "C" {
void I2C1_EV_IRQHandler()
{
  otto::mcu::instances::i2c1.irq_handle_ev();
}
void I2C1_ER_IRQHandler()
{
  otto::mcu::instances::i2c1.irq_handle_er();
}
}
