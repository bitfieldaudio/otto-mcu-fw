#pragma once

#include <chrono>

namespace otto::mcu {

  struct timer_clock {
    using duration = std::chrono::microseconds;
    using rep = duration::rep;
    using period = duration::period;
    using time_point = std::chrono::time_point<timer_clock, duration>;

    static constexpr bool is_steady = true;

    static time_point now() noexcept;
    static bool init();
  };

  struct hal_tick_clock {
    using duration = std::chrono::milliseconds;
    using rep = duration::rep;
    using period = duration::period;
    using time_point = std::chrono::time_point<hal_tick_clock, duration>;

    static constexpr bool is_steady = true;

    static bool init()
    {
      return true;
    }
    static time_point now() noexcept;
  };

  struct systick_clock {
    using duration = std::chrono::microseconds;
    using rep = duration::rep;
    using period = duration::period;
    using time_point = std::chrono::time_point<timer_clock, duration>;

    static constexpr bool is_steady = true;

    static time_point now() noexcept;
    static time_point now_irs() noexcept;
    static bool init();
  };

  using clock = systick_clock;
} // namespace otto::mcu
