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
      {Key::seq0, Key::channel2, Key::channel5, Key::channel8, Key::twist1, Key::sends, Key::blue_enc_click,
       Key::sampler},
      {Key::channel0, Key::channel3, Key::channel6, Key::channel9, Key::fx2, Key::fx1, Key::yellow_enc_click,
       Key::looper},
      {Key::channel1, Key::channel4, Key::channel7, Key::seq15, Key::mixer, Key::unassigned_c, Key::none,
       Key::sequencer},
      {Key::seq1, Key::seq6, Key::seq11, Key::unassigned_d, Key::play, Key::envelope, Key::red_enc_click, Key::synth},
      {Key::seq2, Key::seq7, Key::seq12, Key::unassigned_e, Key::twist2, Key::unassigned_a, Key::none, Key::none},
      {Key::seq3, Key::seq8, Key::seq13, Key::slots, Key::minus, Key::external, Key::none, Key::arp},
      {Key::seq4, Key::seq9, Key::seq14, Key::unassigned_f, Key::record, Key::unassigned_b, Key::green_enc_click,
       Key::settings},
      {Key::seq5, Key::seq10, Key::none, Key::shift, Key::plus, Key::voices, Key::none, Key::master},
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
      {Key::shift, 26},
      {Key::unassigned_d, 27},
      {Key::unassigned_e, 28},
      {Key::unassigned_f, 29},
      {Key::slots, 30},
      {Key::twist1, 31},
      {Key::twist2, 32},
      {Key::play, 33},
      {Key::record, 34},
      {Key::mixer, 35},
      {Key::unassigned_c, 36},
      {Key::unassigned_b, 37},
      {Key::unassigned_a, 38},
      {Key::voices, 39},
      {Key::envelope, 40},
      {Key::external, 41},
      {Key::sends, 42},
      {Key::fx1, 43},
      {Key::fx2, 44},
      {Key::minus, 45},
      {Key::plus, 46},
      {Key::synth, 47},
      {Key::arp, 48},
      {Key::sampler, 49},
      {Key::sequencer, 50},
      {Key::looper, 51},
      {Key::settings, 52},
      {Key::master, 53},
      {Key::blue_enc_click, 255},
      {Key::green_enc_click, 255},
      {Key::yellow_enc_click, 255},
      {Key::red_enc_click, 255},
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
  co_await test_leds();
}


Task test_enc_colors()
{
  auto color = ws2812b::RGBColor{128, 128, 128};
  while (true) {
    co_await main_loop.suspend_for(1ms);
    bool did_change = false;
    for (int i = 0; i < 4; i++) {
      auto v = encoders[i].grab_value();
      if (v == 0) continue;
      did_change = true;
      if (i == 0) {
        color.b = std::clamp(int(color.b) + v, 0, 255);
      } else if (i == 1) {
        color.g = std::clamp(int(color.g) + v, 0, 255);
      } else if (i == 3) {
        color.r = std::clamp(int(color.r) + v, 0, 255);
      } else if (i == 2) {
        color.r = std::clamp(int(color.r) + v, 0, 255);
        color.g = std::clamp(int(color.g) + v, 0, 255);
        color.b = std::clamp(int(color.b) + v, 0, 255);
      }
    }
    if (did_change)
      for (auto&& led : leds) led = color;
  }
}

Task heartbeat(clock::duration interval = 1s)
{
  uint8_t i = 1;
  while (true) {
    i2c1.transmit(make_heartbeat().to_array());
    co_await instances::main_loop.suspend_for(interval);
    i++;
  }
}

Task heartbeat_blocking(clock::duration interval = 1s)
{
  uint8_t i = 1;
  while (true) {
    i2c1.transmit(make_heartbeat().to_array());
    leds[0] = ws2812b::colors[i % ws2812b::colors.size()];
    leds.maybe_update();
    auto time = clock::now() + interval;
    while (clock::now() < time)
      ;
    i++;
  }
}

namespace otto::mcu::power {
  enum struct State { on, shutdown, off } state = State::off;
  GpioPin power_switch = GPIO_PIN(PWR_BUTTON);
  GpioPin rpi_power = GPIO_PIN(PI_PWR_EN);
  GpioPin led_power = GPIO_PIN(LED_PWR_EN);

  Task shutdown()
  {
    state = State::shutdown;
    // Wait 5 seconds for rpi to shutdown
    // TODO: Actual communication
    i2c1.transmit(Packet{Command::shutdown}.to_array());
    co_await instances::main_loop.suspend_for(5s);
    rpi_power.write(false);
    led_power.write(false);
    status_led.write(false);
    state = State::off;
  }

  Task poll(clock::duration interval = 250ms)
  {
    while (true) {
      auto btn_state = power_switch.read();
      if (state == State::off) {
        if (btn_state) {
          rpi_power.write(true);
          led_power.write(true);
          status_led.write(true);
          state = State::on;
        }
      } else if (state == State::on) {
        if (!btn_state) co_await shutdown();
      }
      co_await instances::main_loop.suspend_for(interval);
    }
  }

  void init()
  {
    power_switch.init(GpioPin::Mode::input, GpioPin::Pull::up);
    rpi_power.init(GpioPin::Mode::output_pp);
    led_power.init(GpioPin::Mode::output_pp);
    poll();
  }
} // namespace otto::mcu::power

GpioPin midi_out = {MIDI_OUT_GPIO_Port, MIDI_OUT_Pin};

extern "C" {

/// Set up semihosting
extern void initialise_monitor_handles(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {}

void OTTO_preinit()
{
  /// If a debugger is connected, enable semihosting
  if (is_debug) initialise_monitor_handles();
}

void OTTO_main_loop()
{
  clock::init();
  i2c1.init();
  leds.init();
  for (auto& enc : encoders) enc.init();
  inputs.init();
  power::init();

  main_loop.schedule(0ms, 500us, [] { inputs.poll(); });
  main_loop.schedule(0ms, 10ms, [] { poll_encoders(); });
  main_loop.schedule(0ms, 500us, [] {
    for (auto& e : encoders) e.poll();
  });
  main_loop.schedule(0ms, 100us, [] { i2c1.poll(); });

  heartbeat(1s);

  i2c1.rx_callback = [](i2c::I2CSlave::PacketData data) {
    auto p = Packet::from_array(data);
    log("→ [%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X]",
        (std::uint8_t) p.cmd, p.data[0], p.data[1], p.data[2], p.data[3], p.data[4], p.data[5], p.data[6], p.data[7],
        p.data[8], p.data[9], p.data[10], p.data[11], p.data[12], p.data[13], p.data[14], p.data[15]);
    switch (p.cmd) {
      case Command::leds_buffer: [[fallthrough]];
      case Command::leds_commit: {
        // There may be up to 4 LED colors in one message
        for (int i = 0; i < 16; i += 4) {
          auto idx = led_map[p.data[i + 0]];
          if (idx < leds.size()) {
            leds[idx] = ws2812b::RGBColor{p.data[i + 1], p.data[i + 2], p.data[i + 3]};
          }
        }
        if (p.cmd == Command::leds_commit) {
          leds.send_update();
        }
      } break;
      default: break;
    }
  };

  // main_loop.schedule(0, 1, [] { leds.maybe_update(); });
  // test_enc_colors();

  log("Starting");
  while (true) {
    main_loop.exec();
  }
}
}
