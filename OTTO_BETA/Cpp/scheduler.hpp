#pragma once

#include "local_vector.hpp"

#include "fixed_size_function.hpp"
#include "stm32f4xx_hal.h"

#include <coroutine>

namespace otto::mcu {
  /// Time-trigger scheduler supporting repeated tasks
  struct Scheduler {
    /// Move-only function with max size of 48
    using Function = fixed_size_function<void(), 64, construct_type::move>;
    /// Move-only function with max size of 32
    using CondRepeatFunction = fixed_size_function<std::uint32_t(), 32, construct_type::move>;
    /// Move only task type
    struct TaskElement {
      /// Function to execute
      Function func;
      /// Absolute time to execute function in milliseconds
      std::uint32_t time;
      /// Upon execution, the task will be requeued in X milliseconds after execution started.
      /// If 0, the task will not be repeated
      std::uint32_t repeat = 0;

      bool operator>(const TaskElement& rhs) const
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
    void schedule(std::uint32_t delay, std::uint32_t repeat, Function f)
    {
      push({std::move(f), HAL_GetTick() + delay, repeat});
    }

    void schedule(std::uint32_t delay, Function f)
    {
      schedule(delay, 0, std::move(f));
    }
    void schedule(Function f)
    {
      schedule(0, 0, std::move(f));
    }

    void schedule_cond_repeat(std::uint32_t delay, CondRepeatFunction f)
    {
      schedule(delay, [this, ff = std::move(f)]() mutable {
        auto repeat = ff();
        if (repeat) {
          schedule_cond_repeat(repeat, std::move(ff));
        }
      });
    }

    void schedule_cond_repeat(CondRepeatFunction f)
    {
      schedule_cond_repeat(0, std::move(f));
    }

    Scheduler& get();

  private:
    struct [[nodiscard("You probably want to `co_await` this object")]] SuspendFor
    {
      uint32_t delay;
      Scheduler& scheduler;

      bool await_ready()
      {
        return false;
      }

      void await_suspend(std::coroutine_handle<> handle)
      {
        scheduler.schedule(delay, [handle] { handle.resume(); });
      }

      void await_resume() {}
    };

  public:
    /// `co_await` the result of this function to suspend a coroutine,
    /// and schedule it for continuation in `ms` milliseconds
    SuspendFor suspend_for(uint32_t ms)
    {
      return {ms, *this};
    }

  private:
    TaskElement pop()
    {
      std::pop_heap(queue_.begin(), queue_.end(), std::greater<TaskElement>());
      TaskElement res = std::move(queue_.back());
      queue_.pop_back();
      return res;
    }

    void push(TaskElement&& t)
    {
      queue_.emplace_back(std::move(t));
      std::push_heap(queue_.begin(), queue_.end(), std::greater<TaskElement>());
    }
    // we cant move out of std::priority_queue on pop, so we implement it ourselves
    util::local_vector<TaskElement, 1024> queue_;
  };

  struct Task {
    struct promise_type {
      using coro_handle = std::coroutine_handle<promise_type>;
      Task get_return_object()
      {
        return *this;
      }
      auto initial_suspend()
      {
        return std::suspend_always();
      }
      auto final_suspend()
      {
        if (next_) next_();
        return std::suspend_never();
      }
      void return_void() {}
      void unhandled_exception() {}

    private:
      friend Task;
      Scheduler::Function next_ = nullptr;
    };

    using coro_handle = promise_type::coro_handle;

    Task(const Task&) = delete;
    Task(Task&& o)
    {
      std::swap(promise_, o.promise_);
      std::swap(to_resume, o.to_resume);
    }

    ~Task()
    {
      if (to_resume) to_resume.resume();
    }

    Task then(Task next) &&
    {
      promise_->next_ = [handle = coro_handle::from_promise(*next.promise_)] { handle.resume(); };
      next.to_resume = nullptr;
      return next;
    }

    void then(Scheduler::Function f) &&
    {
      promise_->next_ = std::move(f);
    }

  private:
    Task(promise_type& promise) : promise_(&promise), to_resume(coro_handle::from_promise(promise)){
    }
    promise_type* promise_;
    coro_handle to_resume = nullptr;
  };

} // namespace otto::mcu
