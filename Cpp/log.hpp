#pragma once

#include <cstdio>
#include <cstdarg>

namespace otto::mcu {

#ifdef NDEBUG
  constexpr bool is_debug = false;
#else
  constexpr bool is_debug = false;
#endif

  [[gnu::format(printf, 1, 2)]] //
  inline void
  log(const char* fmt, ...)
  {
    if constexpr (!is_debug) return;
    char buf[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, 1024, fmt, args);
    va_end(args);
    // {GRAY}[LOG]{RESET} message
    printf("\u001b[30;1m[LOG]\u001b[0m %s\n", buf);
  }
} // namespace otto::mcu
