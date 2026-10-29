#pragma once
#include "setpoint_buffer.hpp"
namespace motion {
using ClockId = std::uint64_t;
struct TrackingTime {
  double seconds{};
  ClockId clock{};
  double uncertainty_s{};
};
struct ExecutionTimingPolicy {
  double max_early_s{}, max_late_s{}, max_clock_uncertainty_s{}, dispatch_latency_bound_s{};
};
enum class ExecutionStatus { Running, Finished, Inactive, Error };
enum class ExecutionFault {
  None,
  InvalidClock,
  ClockRegression,
  ExcessUncertainty,
  DeadlineMissed,
  Underflow,
  InvalidCommand
};
template <std::size_t N> class DeadlineMotionExecutor {
public:
  DeadlineMotionExecutor(FixedJointCommand<N> initial, std::shared_ptr<const IExecutionSchedule> schedule,
                         ClockId clock, ExecutionTimingPolicy policy)
      : schedule_(std::move(schedule)), clock_(clock), policy_(policy), initial_(initial) {
    if (!schedule_ || !clock || schedule_->initial_state().q.size() != N || initial.tick != 0)
      throw std::invalid_argument("Invalid execution schedule");
    for (double x : {policy.max_early_s, policy.max_late_s, policy.max_clock_uncertainty_s,
                     policy.dispatch_latency_bound_s})
      if (!std::isfinite(x) || x < 0)
        throw std::invalid_argument("Invalid timing policy");
    const double window = policy.max_early_s + policy.max_late_s;
    if (window <= 0 || window >= schedule_->cycle_s() ||
        2 * policy.max_clock_uncertainty_s + policy.dispatch_latency_bound_s > window)
      throw std::invalid_argument("Overlapping timing windows");
    limits_ = schedule_->joint_limits();
    epoch_ = schedule_->initial_state().timestamp_s;
    cycle_ = schedule_->cycle_s();
    end_ = schedule_->total_ticks();
    boundaries_ = schedule_->boundary_ticks();
  }
  ExecutionFault fault() const noexcept { return fault_; }
  template <std::size_t Capacity>
  ExecutionStatus poll(SpscSetpointBuffer<N, Capacity> &ring, TrackingTime now,
                       FixedJointCommand<N> &out) noexcept {
    return poll_impl(ring, now, out, [now] { return now; }, false);
  }
  template <std::size_t Capacity, class Clock>
  ExecutionStatus poll_clock(SpscSetpointBuffer<N, Capacity> &ring, Clock &clock,
                             FixedJointCommand<N> &out) noexcept {
    return poll_impl(ring, clock.now(), out, [&clock] { return clock.now(); }, true);
  }

private:
  bool clock_valid(TrackingTime t) noexcept {
    if (!std::isfinite(t.seconds) || !std::isfinite(t.uncertainty_s) || t.uncertainty_s < 0 ||
        t.clock != clock_) {
      fault_ = ExecutionFault::InvalidClock;
      return false;
    }
    if (t.uncertainty_s > policy_.max_clock_uncertainty_s) {
      fault_ = ExecutionFault::ExcessUncertainty;
      return false;
    }
    if (last_ && t.seconds < *last_) {
      fault_ = ExecutionFault::ClockRegression;
      return false;
    }
    last_ = t.seconds;
    return true;
  }
  template <std::size_t Capacity, class Observe>
  ExecutionStatus poll_impl(SpscSetpointBuffer<N, Capacity> &ring, TrackingTime now,
                            FixedJointCommand<N> &out, Observe observe, bool second) noexcept {
    if (fault_ != ExecutionFault::None)
      return ExecutionStatus::Error;
    if (finished_)
      return ExecutionStatus::Inactive;
    if (!clock_valid(now))
      return ExecutionStatus::Error;
    const double due = epoch_ + schedule_detail::tick_time(next_, cycle_);
    const auto window = [&](TrackingTime t) {
      return static_cast<long double>(t.seconds) - t.uncertainty_s >=
                 static_cast<long double>(due) - policy_.max_early_s &&
             static_cast<long double>(t.seconds) + t.uncertainty_s + policy_.dispatch_latency_bound_s <=
                 static_cast<long double>(due) + policy_.max_late_s;
    };
    if (static_cast<long double>(now.seconds) + now.uncertainty_s + policy_.dispatch_latency_bound_s >
        static_cast<long double>(due) + policy_.max_late_s) {
      fault_ = ExecutionFault::DeadlineMissed;
      return ExecutionStatus::Error;
    }
    if (!window(now))
      return ExecutionStatus::Inactive;
    FixedJointCommand<N> candidate;
    if (!ring.try_pop(candidate)) {
      fault_ = ExecutionFault::Underflow;
      return ExecutionStatus::Error;
    }
    bool valid = buffer_detail::finite(candidate) && candidate.tick == next_ &&
                 candidate.program_time_s == schedule_detail::tick_time(next_, cycle_) &&
                 candidate.timestamp_s == due && candidate.terminal == (next_ == end_);
    for (std::size_t i = 0; i < N; ++i)
      if (candidate.q[i] < limits_.min_position[i] || candidate.q[i] > limits_.max_position[i] ||
          std::abs(candidate.dq[i]) > limits_.max_velocity[i] + 1e-8 ||
          std::abs(candidate.ddq[i]) > limits_.max_acceleration[i] + 1e-8)
        valid = false;
    if (next_ == 0 && candidate != initial_)
      valid = false;
    if (boundary_ < boundaries_.size() && boundaries_[boundary_] == next_) {
      if (!buffer_detail::rest(candidate))
        valid = false;
      ++boundary_;
    }
    if (candidate.terminal && !buffer_detail::rest(candidate))
      valid = false;
    if (!valid) {
      fault_ = ExecutionFault::InvalidCommand;
      return ExecutionStatus::Error;
    }
    if (second) {
      const auto late = observe();
      if (!clock_valid(late))
        return ExecutionStatus::Error;
      if (!window(late)) {
        fault_ = ExecutionFault::DeadlineMissed;
        return ExecutionStatus::Error;
      }
    }
    out = candidate;
    ++next_;
    finished_ = candidate.terminal;
    return finished_ ? ExecutionStatus::Finished : ExecutionStatus::Running;
  }
  std::shared_ptr<const IExecutionSchedule> schedule_;
  ClockId clock_;
  ExecutionTimingPolicy policy_;
  FixedJointCommand<N> initial_;
  JointLimits limits_;
  double epoch_{}, cycle_{};
  std::uint64_t next_{}, end_{};
  std::vector<std::uint64_t> boundaries_;
  std::size_t boundary_{};
  std::optional<double> last_;
  ExecutionFault fault_{ExecutionFault::None};
  bool finished_{};
};
} // namespace motion
