#include "clock.hpp"

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_tim_ex.h"

#undef SET_BIT
#undef CLEAR_BIT
#define SET_BIT(REG, BIT) ((REG) = (REG) | (BIT))
#define CLEAR_BIT(REG, BIT) ((REG) = (REG) & ~(BIT))

static std::uint32_t get_APB1_timer_clkfreq()
{
  /* Get PCLK1 frequency */
  std::uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();

  /* Get PCLK1 prescaler */
  if ((RCC->CFGR & RCC_CFGR_PPRE1) == 0) {
    /* PCLK1 prescaler equal to 1 => TIMCLK = PCLK1 */
    return (pclk1);
  } else {
    /* PCLK1 prescaler different from 1 => TIMCLK = 2 * PCLK1 */
    return (2 * pclk1);
  }
}

namespace otto::mcu {
  static TIM_HandleTypeDef clock_timer = {.Instance = TIM2};

  bool timer_clock::init()
  {
    std::uint32_t gu32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    clock_timer.Instance = TIM2;
    clock_timer.Init.Prescaler = 96 - 1;
    clock_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    // Only TIM2 and TIM5 are 32 bit
    clock_timer.Init.Period = 0xFFFFFFFF;
    clock_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    clock_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&clock_timer) != HAL_OK) {
      return false;
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&clock_timer, &sClockSourceConfig) != HAL_OK) {
      return false;
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&clock_timer, &sMasterConfig) != HAL_OK) {
      return false;
    }

    __HAL_RCC_TIM2_CLK_ENABLE();
    // HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(TIM2_IRQn);

    HAL_TIM_Base_Start(&clock_timer);

    return true;
  }

  static std::int64_t clock_val = 0;

  timer_clock::time_point timer_clock::now() noexcept
  {
    std::uint32_t us32 = clock_timer.Instance->CNT;
    std::int64_t us = us32;
    // Wraps ~ once per hour, we assume this function is called more than once per hour
    while (us < clock_val) {
      us += 0x100000000;
    }
    clock_val = us;
    return timer_clock::time_point(std::chrono::microseconds(us));
  }

  hal_tick_clock::time_point hal_tick_clock::now() noexcept
  {
    return time_point(duration(HAL_GetTick()));
  }

  static volatile std::uint64_t systick_clock_ms = 0;

  bool systick_clock::init()
  {
    return true;
  }

  systick_clock::time_point systick_clock::now() noexcept
  {
    std::uint64_t ms;
    std::uint32_t st;
    do {
      ms = systick_clock_ms;
      st = SysTick->VAL;
      asm volatile("nop");
      asm volatile("nop");
    } while (ms != systick_clock_ms);

    auto us = ms * 1000ULL - st / ((SysTick->LOAD + 1) / 1000ULL);
    return timer_clock::time_point(std::chrono::microseconds(us));
  }

  systick_clock::time_point systick_clock::now_irs() noexcept
  {
    std::uint32_t st = SysTick->VAL;
    std::uint32_t pending = SCB->ICSR & SCB_ICSR_PENDSTSET_Msk;
    std::uint64_t ms = systick_clock_ms;

    if (pending == 0) ms++;

    auto us = ms * 1000 - st / ((SysTick->LOAD + 1) / 1000);
    return timer_clock::time_point(std::chrono::microseconds(us));
  }
} // namespace otto::mcu

extern "C" {
void SysTick_Handler()
{
  otto::mcu::systick_clock_ms = otto::mcu::systick_clock_ms + 1;
  HAL_IncTick();
}
}
