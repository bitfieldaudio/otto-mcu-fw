#pragma once

#include "clock.hpp"
#include "local_vector.hpp"

#include "fixed_size_function.hpp"
#include "stm32f4xx_hal.h"

#include <chrono>
#include <compare>
#include <coroutine>
#include <optional>

// Makes clangd shut up about not having these in std::experimental
namespace std::experimental {
  using std::coroutine_handle;
  using std::coroutine_traits;
} // namespace std::experimental

namespace otto::mcu {
  using namespace std::chrono_literals;

  /// Time-trigger scheduler supporting repeated tasks
  struct Scheduler {
    /// Move-only function with max size of 32
    using Function = fixed_size_function<void(), 32, construct_type::move>;
    /// Move only task type
    struct TaskElement {
      /// Function to execute
      Function func;
      /// Absolute time to execute function in milliseconds
      clock::time_point time;
      /// Upon execution, the task will be requeued in X milliseconds after execution started.
      std::optional<clock::duration> repeat = std::nullopt;

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
      auto time = clock::now();
      if (queue_.empty()) return false;
      if (queue_.front().time <= time) {
        auto task = pop();
        assert_param(task.func);
        task.func();
        if (task.repeat) {
          task.time = time + *task.repeat;
          push(std::move(task));
        }
        return true;
      }
      return false;
    }

    /// Schedule a function with optional initial delay and repeat times
    ///
    /// if `repeat`, then f will, upon completion, be scheduled
    /// again with a delay of `repeat`
    void schedule(clock::duration delay, std::optional<clock::duration> repeat, Function f)
    {
      push({std::move(f), clock::now() + delay, repeat});
    }

    void schedule(clock::duration delay, Function f)
    {
      schedule(delay, std::nullopt, std::move(f));
    }
    void schedule(Function f)
    {
      schedule(clock::duration::zero(), std::nullopt, std::move(f));
    }

  private:
    struct [[nodiscard("You probably want to `co_await` this object")]] SuspendFor
    {
      clock::duration delay;
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
    /// and schedule it for continuation after delay
    SuspendFor suspend_for(clock::duration delay)
    {
      return {delay, *this};
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
