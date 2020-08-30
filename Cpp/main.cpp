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
      {Key::seq5, Key::seq10, Key::none, Key::shift, Key::play, Key::external, Key::none, Key::master},
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

  constexpr std::array<std::uint8_t, 59> led_map = [] {
    std::array<std::pair<Key, std::uint8_t>, 58> pairs = {{
      {Key::channel0, 0},
      {Key::channel1, 1},
      {Key::channel2, 2},
      {Key::channel3, 3},
      {Key::channel4, 4},
      {Key::channel5, 5},
      {Key::channel6, 6},
      {Key::channel7, 7},
      {Key::channel8, 8},
      {Key::channel9, 9},
      {Key::seq0, 10},
      {Key::seq1, 11},
      {Key::seq2, 12},
      {Key::seq3, 13},
      {Key::seq4, 14},
      {Key::seq5, 15},
      {Key::seq6, 16},
      {Key::seq7, 17},
      {Key::seq8, 18},
      {Key::seq9, 19},
      {Key::seq10, 20},
      {Key::seq11, 21},
      {Key::seq12, 22},
      {Key::seq13, 23},
      {Key::seq14, 24},
      {Key::seq15, 25},
      {Key::blue_enc_click, 255},
      {Key::green_enc_click, 255},
      {Key::yellow_enc_click, 255},
      {Key::red_enc_click, 255},
      {Key::shift, 26},
      {Key::sends, 43},
      {Key::plus, 33},
      {Key::routing, 40},
      {Key::minus, 34},
      {Key::fx1, 51},
      {Key::fx2, 44},
      {Key::master, 53},
      {Key::play, 46},
      {Key::record, 45},
      {Key::arp, 48},
      {Key::slots, 35},
      {Key::twist1, 31},
      {Key::twist2, 32},
      {Key::looper, 38},
      {Key::external, 39},
      {Key::sampler, 42},
      {Key::envelope, 50},
      {Key::voices, 47},
      {Key::settings, 52},
      {Key::sequencer, 41},
      {Key::synth, 49},
      {Key::unassigned_a, 37},
      {Key::unassigned_b, 36},
      {Key::unassigned_c, 27},
      {Key::unassigned_d, 28},
      {Key::unassigned_e, 29},
      {Key::unassigned_f, 30},
    }};
    std::array<std::uint8_t, 59> res;
    for (auto& i : res) {
      i = 255;
    }
    for (auto [k, v] : pairs) {
      res[static_cast<std::uint8_t>(k)] = v;
    }

    return res;
  }();

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

void poll_encoders()
{
  Packet p;
  for (int i = 0; i < 4; i++) {
    auto v = encoders[i].grab_value();
    if (v == 0) continue;
    p.cmd = Command::encoder_events;
    p.data[i] = static_cast<std::uint8_t>(v);
  }
  if (p.cmd != Command::none) transmit(p);
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
  for (auto& e : encoders) e.irq_checked(GPIO_Pin);
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
  for (auto& enc : encoders) enc.init();
  inputs.init();

  main_loop.schedule(0, 1, [] { leds.maybe_update(); });
  main_loop.schedule(0, 1, [] { inputs.poll(); });
  main_loop.schedule(0, 1, [] { poll_encoders(); });
  main_loop.schedule(0, 1, [] { i2c1.poll(); });

  i2c1.rx_callback = [](i2c::I2CSlave::PacketData data) {
    auto p = Packet::from_array(data);
    switch (p.cmd) {
      case Command::led_set: {
        auto idx = led_map[p.data[0]];
        if (idx > 54) break;
        leds[idx] = ws2812b::RGBColor{p.data[1], p.data[2], p.data[3]};
      } break;
      default: break;
    }
  };

  while (true) {
    main_loop.exec();
  }
}
}
