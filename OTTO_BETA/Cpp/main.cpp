#include "encoder.hpp"
#include "i2c.hpp"
#include "instances.hpp"
#include "keys.hpp"
#include "log.hpp"
#include "scheduler.hpp"
#include "ws2812b.hpp"

#include "stm32f4xx_hal_i2c.h"

namespace otto::mcu::instances {
  Scheduler main_loop;
  ws2812b::Ws2812bArray leds = {hspi3, 54};
  i2c::I2CSlave i2c1;

  InputManager inputs = InputManager::KeyMatrix{
    .table = {{
      {Key::seq0, Key::channel2, Key::channel5, Key::channel8, Key::twist1, Key::sampler, Key::blue_enc_click,
       Key::synth},
      {Key::channel0, Key::channel3, Key::channel6, Key::channel9, Key::fx2, Key::sends, Key::yellow_enc_click,
       Key::fx1},
      {Key::channel1, Key::channel4, Key::channel7, Key::seq15, Key::slots, Key::unassigned_b, Key::none,
       Key::envelope},
      {Key::seq1, Key::seq6, Key::seq11, Key::unassigned_c, Key::plus, Key::routing, Key::red_enc_click, Key::voices},
      {Key::seq2, Key::seq7, Key::seq12, Key::unassigned_d, Key::twist2, Key::looper, Key::none, Key::none},
      {Key::seq3, Key::seq8, Key::seq13, Key::unassigned_f, Key::record, Key::sequencer, Key::none, Key::arp},
      {Key::seq4, Key::seq9, Key::seq14, Key::unassigned_e, Key::minus, Key::unassigned_a, Key::green_enc_click,
       Key::settings},
      {Key::seq5, Key::seq10, Key::none, Key::shift, Key::plus, Key::external, Key::none},
    }},
    .row_pins = {{
      GPIO_PIN(ROW_1),
      GPIO_PIN(ROW_2),
      GPIO_PIN(ROW_3),
      GPIO_PIN(ROW_4),
      GPIO_PIN(ROW_5),
      GPIO_PIN(ROW_6),
      GPIO_PIN(ROW_7),
      GPIO_PIN(ROW_8),
    }},
    .col_pins = {{
      GPIO_PIN(COL_1),
      GPIO_PIN(COL_2),
      GPIO_PIN(COL_3),
      GPIO_PIN(COL_4),
      GPIO_PIN(COL_5),
      GPIO_PIN(COL_6),
      GPIO_PIN(COL_7),
      GPIO_PIN(COL_8),
    }},
  };

  std::array<Encoder, 4> encoders = {{
    {GPIO_PIN(ENC_1_A), GPIO_PIN(ENC_1_B)},
    {GPIO_PIN(ENC_2_A), GPIO_PIN(ENC_2_B)},
    {GPIO_PIN(ENC_3_A), GPIO_PIN(ENC_3_B)},
    {GPIO_PIN(ENC_4_A), GPIO_PIN(ENC_4_B)},
  }};

  const GpioPin status_led = {STATUS_LED_GPIO_Port, STATUS_LED_Pin, true};

} // namespace otto::mcu::instances

using namespace otto::mcu;
using namespace otto::mcu::instances;

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

Task test_leds()
{
  co_await led_pulse_colors(leds);
  co_await led_cascade_colors(leds);
  leds.clear();
}

namespace otto::mcu::power {
  bool state = false;
  GpioPin power_switch = GPIO_PIN(PWR_BUTTON);
  GpioPin rpi_power = GPIO_PIN(PI_PWR_EN);
  GpioPin led_power = GPIO_PIN(LED_PWR_EN);
  GpioPin reg_iout = GPIO_PIN(REG_5V_IOUT);
  GpioPin reg_pwm = GPIO_PIN(REG_5V_PWM);

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
    rpi_power.init(GpioPin::Mode::output_pp);
    led_power.init(GpioPin::Mode::output_pp);
    reg_iout.init(GpioPin::Mode::analog);
    reg_pwm.init(GpioPin::Mode::output_pp);
    reg_pwm.write(1);
    main_loop.schedule(0, 500, poll);
    state_changed();
  }
} // namespace otto::mcu::power

GpioPin midi_out = {MIDI_OUT_GPIO_Port, MIDI_OUT_Pin};

extern "C" {

/// Set up semihosting
extern void initialise_monitor_handles(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  blue_encoder.irq_checked(GPIO_Pin);
  green_encoder.irq_checked(GPIO_Pin);
  yellow_encoder.irq_checked(GPIO_Pin);
  red_encoder.irq_checked(GPIO_Pin);
}

void OTTO_preinit()
{
  /// If a debugger is connected, enable semihosting
  if (is_debug) initialise_monitor_handles();
}

void OTTO_main_loop()
{
  power::init();
  i2c1.init();
  leds.init();
  main_loop.schedule(0, 20, [] { leds.maybe_update(); });
  blue_encoder.init();
  green_encoder.init();
  yellow_encoder.init();
  red_encoder.init();
  inputs.init();
  main_loop.schedule(0, 1, [] { inputs.poll(); });
  // Test
  i2c1.rx_callback = [](std::span<const std::uint8_t> data) {
    i2c::I2CSlave::PacketData packet;
    std::ranges::fill(packet, 0);
    std::ranges::copy(data, packet.begin());
    i2c1.transmit(packet);
  };

  while (true) {
    main_loop.exec();
  }
}
}
