#pragma once

#include "local_vector.hpp"

#include "fixed_size_function.hpp"
#include "stm32f4xx_hal.h"

namespace otto::mcu {

  /// Time-trigger scheduler supporting repeated tasks
  struct Scheduler {
    /// Move-only function with max size of 48
    using Function = fixed_size_function<void(), 64, construct_type::move>;
    /// Move-only function with max size of 32
    using CondRepeatFunction = fixed_size_function<std::uint32_t(), 32, construct_type::move>;
    /// Move only task type
    struct Task {
      /// Function to execute
      Function func;
      /// Absolute time to execute function in milliseconds
      std::uint32_t time;
      /// Upon execution, the task will be requeued in X milliseconds after execution started.
      /// If 0, the task will not be repeated
      std::uint32_t repeat = 0;

      bool operator>(const Task& rhs) const
      {
        return time > rhs.time;
      }
    };

    /// Execute a task if available.
    ///
    /// Returns true if a task was executed
    bool exec()
    {
      auto time = HAL_GetTick();
      if (queue_.empty()) return false;
      if (queue_.front().time <= time) {
        auto task = pop();
        assert_param(task.func);
        task.func();
        if (task.repeat > 0) {
          task.time = time + task.repeat;
          push(std::move(task));
        }
        return true;
      }
      return false;
    }

    /// Schedule function `f` to be run at least `delay` ms from now, repeating each `repeat` ms.
    /// 
    /// Repeats do not accumulate, i.e, after each execution of `f`, it simply reschedules with a delay of
    /// `repeat`.
    /// 
    /// `repeat == 0` means no repetition
    void schedule(Function f, std::uint32_t delay = 0, std::uint32_t repeat = 0)
    {
      push({std::move(f), HAL_GetTick() + delay, repeat});
    }

    void schedule_cond_repeat(CondRepeatFunction f, std::uint32_t delay = 0) {
      schedule([this, ff = std::move(f)] () mutable {
        auto repeat = ff();
        if (repeat) {
          schedule_cond_repeat(std::move(ff), repeat);
        }
      }, delay);
    }

  private:
    Task pop()
    {
      std::pop_heap(queue_.begin(), queue_.end(), std::greater<Task>());
      Task res = std::move(queue_.back());
      queue_.pop_back();
      return res;
    }

    void push(Task&& t)
    {
      queue_.emplace_back(std::move(t));
      std::push_heap(queue_.begin(), queue_.end(), std::greater<Task>());
    }
    // we cant move out of std::priority_queue on pop, so we implement it ourselves
    util::local_vector<Task, 1024> queue_;
  };

} // namespace otto::mcu
