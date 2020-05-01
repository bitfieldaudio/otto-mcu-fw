#include "encoder.hpp"
#include "keys.hpp"
#include "otto.hpp"
#include "scheduler.hpp"
#include "ws2812b.hpp"

#include "stm32f4xx_hal_i2c.h"

namespace otto::mcu {
  Scheduler main_loop;
}

using namespace otto::mcu;

ws2812b::Ws2812bArray<54> leds = {hspi3};

struct KeyData {
  int led_idx = -1;
};

using KeyMatrix = keys::KeyMatrix<KeyData, 8, 8>;

KeyMatrix key_table = {
  .table = {{
    KeyMatrix::Row{{{{10}}, {{2}}, {{5}}, {{8}}, {{31}}, {{42}}, {{}}, {{49}}}},
    KeyMatrix::Row{{{{0}}, {{3}}, {{6}}, {{9}}, {{44}}, {{43}}, {{}}, {{51}}}},
    KeyMatrix::Row{{{{1}}, {{4}}, {{7}}, {{25}}, {{35}}, {{36}}, {{}}, {{50}}}},
    KeyMatrix::Row{{{{11}}, {{16}}, {{21}}, {{27}}, {{33}}, {{40}}, {{}}, {{47}}}},
    KeyMatrix::Row{{{{12}}, {{17}}, {{22}}, {{28}}, {{32}}, {{38}}, {{}}, {{}}}},
    KeyMatrix::Row{{{{13}}, {{18}}, {{23}}, {{30}}, {{45}}, {{41}}, {{}}, {{48}}}},
    KeyMatrix::Row{{{{14}}, {{19}}, {{24}}, {{29}}, {{34}}, {{37}}, {{}}, {{52}}}},
    KeyMatrix::Row{{{{15}}, {{20}}, {{}}, {{26}}, {{46}}, {{39}}, {{}}, {{53}}}},
  }},
  .row_pins = {{
    {ROW_1_GPIO_Port, ROW_1_Pin},
    {ROW_2_GPIO_Port, ROW_2_Pin},
    {ROW_3_GPIO_Port, ROW_3_Pin},
    {ROW_4_GPIO_Port, ROW_4_Pin},
    {ROW_5_GPIO_Port, ROW_5_Pin},
    {ROW_6_GPIO_Port, ROW_6_Pin},
    {ROW_7_GPIO_Port, ROW_7_Pin},
    {ROW_8_GPIO_Port, ROW_8_Pin},
  }},
  .col_pins = {{
    {COL_1_GPIO_Port, COL_1_Pin},
    {COL_2_GPIO_Port, COL_2_Pin},
    {COL_3_GPIO_Port, COL_3_Pin},
    {COL_4_GPIO_Port, COL_4_Pin},
    {COL_5_GPIO_Port, COL_5_Pin},
    {COL_6_GPIO_Port, COL_6_Pin},
    {COL_7_GPIO_Port, COL_7_Pin},
    {COL_8_GPIO_Port, COL_8_Pin},
  }},
};

Encoder blue_encoder = {{ENC_1_A_GPIO_Port, ENC_1_A_Pin}, {ENC_1_B_GPIO_Port, ENC_1_B_Pin}};
Encoder green_encoder = {{ENC_2_A_GPIO_Port, ENC_2_A_Pin}, {ENC_2_B_GPIO_Port, ENC_2_B_Pin}};
Encoder yellow_encoder = {{ENC_3_A_GPIO_Port, ENC_3_A_Pin}, {ENC_3_B_GPIO_Port, ENC_3_B_Pin}};
Encoder red_encoder = {{ENC_4_A_GPIO_Port, ENC_4_A_Pin}, {ENC_4_B_GPIO_Port, ENC_4_B_Pin}};

void test_leds()
{
  led_pulse_colors(leds);
  led_cascade_colors(leds);
}

/// Tests keys by updating the corresponding LED for each key.
void test_keys()
{
  key_table.scan();
  for (const auto& row : key_table.table) {
    for (auto& cell : row) {
      if (cell.data.led_idx < 0) continue;
      leds[cell.data.led_idx] = cell.is_down ? ws2812b::RGBColor{0x00, 0x80, 0x0} : ws2812b::RGBColor{0x00, 0x00, 0x00};
    }
  }
  leds.maybe_update();
  HAL_Delay(5);
}

void test_encoders()
{
  blue_encoder.handler = [] {
    int led_idx = std::clamp(blue_encoder.value + 0, 0, 15) + 10;
    leds.clear();
    leds[led_idx] = {0x00, 0x00, 0x80};
  };
  green_encoder.handler = [] {
    int led_idx = std::clamp(green_encoder.value + 0, 0, 15) + 10;
    leds.clear();
    leds[led_idx] = {0x00, 0x80, 0x00};
  };
  yellow_encoder.handler = [] {
    int led_idx = std::clamp(yellow_encoder.value + 0, 0, 15) + 10;
    leds.clear();
    leds[led_idx] = {0x80, 0x80, 0x00};
  };
  red_encoder.handler = [] {
    int led_idx = std::clamp(red_encoder.value + 0, 0, 15) + 10;
    leds.clear();
    leds[led_idx] = {0x80, 0x00, 0x00};
  };
}

namespace otto::mcu::power {
  bool state = false;
  GpioPin power_switch = {PWR_BUTTON_GPIO_Port, PWR_BUTTON_Pin};
  GpioPin rpi_power = {PI_PWR_EN_GPIO_Port, PI_PWR_EN_Pin};
  GpioPin led_power = {LED_PWR_EN_GPIO_Port, LED_PWR_EN_Pin};

  void state_changed()
  {
    rpi_power.write(state);
    led_power.write(state);
    status_led.write(state);
  }

  void poll()
  {
    auto new_state = power_switch.read();
    if (new_state == state) return;
    state = new_state;
    state_changed();
  }

  void init()
  {
    power_switch.init(GpioPin::Mode::input, GpioPin::Pull::up);
    main_loop.schedule(poll, 0, 500);
    state_changed();
  }
} // namespace otto::mcu::power

extern "C" {
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  blue_encoder.irq_checked(GPIO_Pin);
  green_encoder.irq_checked(GPIO_Pin);
  yellow_encoder.irq_checked(GPIO_Pin);
  red_encoder.irq_checked(GPIO_Pin);
}

void OTTO_main_loop()
{
  key_table.init();
  leds.init();
  main_loop.schedule([] { leds.maybe_update(); }, 0, 20);
  blue_encoder.init();
  green_encoder.init();
  yellow_encoder.init();
  red_encoder.init();
  power::init();

  test_encoders();
  // ws2812b::led_cascade_colors(leds);

  while (true) {
    main_loop.exec();
  }
}
}
