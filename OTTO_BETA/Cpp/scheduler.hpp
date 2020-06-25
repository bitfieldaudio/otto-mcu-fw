#pragma once

#include "local_vector.hpp"

#include "fixed_size_function.hpp"
#include "stm32f4xx_hal.h"

#include <chrono>
#include <compare>
#include <coroutine>

// Makes clangd shut up about not having these in std::experimental
namespace std::experimental {
  using std::coroutine_handle;
  using std::coroutine_traits;
} // namespace std::experimental

namespace otto::mcu {
  /// Time-trigger scheduler supporting repeated tasks
  struct Scheduler {
    /// Move-only function with max size of 32
    using Function = fixed_size_function<void(), 32, construct_type::move>;
    /// Move only task type
    struct TaskElement {
      /// Function to execute
      Function func;
      /// Absolute time to execute function in milliseconds
      std::uint32_t time;
      /// Upon execution, the task will be requeued in X milliseconds after execution started.
      /// If 0, the task will not be repeated
      int repeat = -1;

      std::strong_ordering operator<=>(const TaskElement& rhs) const
      {
        return time <=> rhs.time;
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
        if (task.repeat >= 0) {
          task.time = time + task.repeat;
          push(std::move(task));
        }
        return true;
      }
      return false;
    }

    /// Schedule a function with optional initial delay and repeat times
    ///
    /// if `repeat >= 0`, then f will, upon completion, be scheduled
    /// again with a delay of `repeat` ms
    void schedule(std::uint32_t delay, int repeat, Function f)
    {
      push({std::move(f), HAL_GetTick() + delay, repeat});
    }

    void schedule(int delay, Function f)
    {
      schedule(delay, -1, std::move(f));
    }
    void schedule(Function f)
    {
      schedule(0, -1, std::move(f));
    }

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

  /// A very basic coroutine return type, that allows awaiting a `Task`
  /// to resume when it's done.
  struct Task {
    struct promise_type {
      using coro_handle = std::coroutine_handle<promise_type>;
      Task get_return_object()
      {
        return *this;
      }
      auto initial_suspend()
      {
        return std::suspend_never();
      }
      auto final_suspend()
      {
        if (next_) next_();
        return std::suspend_never();
      }
      void return_void() {}
      void unhandled_exception() {}

      auto await_transform(Task awaitable)
      {
        awaitable.promise_->next_ = coro_handle::from_promise(*this);
        return std::suspend_always();
      }

      decltype(auto) await_transform(auto&& awaitable)
      {
        return std::forward<decltype(awaitable)>(awaitable);
      }

    private:
      coro_handle next_ = nullptr;
    };

    using coro_handle = promise_type::coro_handle;

  private:
    Task(promise_type& promise) : promise_(&promise) {}
    promise_type* promise_;
  };

} // namespace otto::mcu
