#pragma once
#include "motion/execution/live_plan_owner.hpp"
#include <utility>
namespace motion {
enum class LiveSessionFaultCode : std::uint64_t { None, Execution, SinkRejected, CommitRejected };
struct LiveSessionFault {
  LiveSessionFaultCode code{LiveSessionFaultCode::None};
  ExecutionFault execution_fault{ExecutionFault::None};
  std::uint64_t generation{}, tick{};
};
/// Construct/destroy outside RT. The host must stop/join every caller before
/// destruction. There is no hidden thread or blocking destructor.
/// One cyclic thread dispatches to a synchronous, bounded, noexcept sink:
///   bool try_write(const LiveJointCommand<N>&) noexcept;
///   void request_stop() noexcept;
/// true means the sink has copied and accepted the whole command irrevocably;
/// false never causes a retry. The sink must not retain a reference to its argument.
template <std::size_t N> class LiveMotionSession {
  static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
  static_assert(std::atomic<LiveSessionState>::is_always_lock_free);
  static_assert(std::atomic<LiveSessionFaultCode>::is_always_lock_free);

public:
  LiveMotionSession(std::shared_ptr<const PreparedLivePlan<N>> initial, ExecutionTimingPolicy timing,
                    std::uint64_t lead = 1)
      : plans_(std::move(initial)), executor_(*plans_.active_plan_, plans_.mailbox_, timing, lead) {}
  LiveMotionSession(const LiveMotionSession &) = delete;
  LiveMotionSession &operator=(const LiveMotionSession &) = delete;
  LivePlanOwner<N> &plans() noexcept { return plans_; }
  const LivePlanOwner<N> &plans() const noexcept { return plans_; }
  std::optional<LiveExecutionSnapshot> snapshot() const noexcept { return executor_.snapshot(); }
  /// Safe concurrent diagnostic read. The record is written once, then immutable.
  std::optional<LiveSessionFault> diagnostic() const noexcept {
    if (fault_code_.load(std::memory_order_acquire) == LiveSessionFaultCode::None)
      return std::nullopt;
    return fault_record_;
  }
  template <class Sink> ExecutionStatus dispatch(ClockTime now, Sink &sink) noexcept {
    return dispatch_impl(sink, [&](LiveJointCommand<N> &command) { return executor_.poll(now, command); });
  }
  template <class Clock, class Sink> ExecutionStatus dispatch_clock(Clock &clock, Sink &sink) noexcept {
    static_assert(noexcept(clock.now()), "The cyclic clock must be noexcept");
    return dispatch_impl(sink,
                         [&](LiveJointCommand<N> &command) { return executor_.poll_clock(clock, command); });
  }

private:
  void record_fault(LiveSessionFaultCode code, std::uint64_t generation, std::uint64_t tick) noexcept {
    if (fault_code_.load(std::memory_order_relaxed) != LiveSessionFaultCode::None)
      return;
    fault_record_ = {code, executor_.fault(), generation, tick};
    fault_code_.store(code, std::memory_order_release);
  }
  template <class Sink> void quiesce(Sink &sink, bool fault) noexcept {
    if (fault || !terminal_delivered_)
      sink.request_stop(); // Host notification, not a physical standstill acknowledgement.
    executor_.shutdown();
    plans_.state_.store(LiveSessionState::Quiescent, std::memory_order_release);
  }
  template <class Sink, class Poll> ExecutionStatus dispatch_impl(Sink &sink, Poll poll) noexcept {
    static_assert(noexcept(sink.try_write(std::declval<const LiveJointCommand<N> &>())),
                  "The cyclic sink must be noexcept");
    static_assert(noexcept(sink.request_stop()), "The cyclic stop notification must be noexcept");
    const auto state = plans_.state();
    if (state == LiveSessionState::Quiescent)
      return fault_code_.load(std::memory_order_relaxed) == LiveSessionFaultCode::None
                 ? ExecutionStatus::Inactive
                 : ExecutionStatus::Error;
    if (state == LiveSessionState::ShutdownRequested) {
      quiesce(sink, false);
      return ExecutionStatus::Inactive;
    }
    if (const auto generation = plans_.cancellation_generation_.exchange(0, std::memory_order_acq_rel))
      executor_.cancel_offer(generation);
    LiveJointCommand<N> command;
    const auto status = poll(command);
    if (status == ExecutionStatus::Error) {
      const auto position = executor_.snapshot(); // Single RT writer, no concurrent publish here.
      record_fault(LiveSessionFaultCode::Execution, executor_.generation(),
                   position ? position->next_tick : 0);
      quiesce(sink, true);
      return ExecutionStatus::Error;
    }
    if (status == ExecutionStatus::Running || status == ExecutionStatus::Finished) {
      if (!sink.try_write(command)) {
        record_fault(LiveSessionFaultCode::SinkRejected, command.generation, command.joints.tick);
        quiesce(sink, true);
        return ExecutionStatus::Error;
      }
      if (!executor_.commit_delivery(command.generation, command.joints.tick)) {
        record_fault(LiveSessionFaultCode::CommitRejected, command.generation, command.joints.tick);
        quiesce(sink, true);
        return ExecutionStatus::Error;
      }
      terminal_delivered_ = status == ExecutionStatus::Finished;
    }
    return status;
  }
  // Declaration order ensures executor destruction precedes mailbox/cache destruction.
  LivePlanOwner<N> plans_;
  LiveMotionExecutor<N> executor_;
  bool terminal_delivered_{};
  LiveSessionFault fault_record_;
  std::atomic<LiveSessionFaultCode> fault_code_{LiveSessionFaultCode::None};
};
} // namespace motion
