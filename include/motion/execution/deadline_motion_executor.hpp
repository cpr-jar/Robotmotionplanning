#pragma once
#include "motion/execution/execution_types.hpp"
#include "motion/execution/setpoint_buffer.hpp"
namespace motion {
template <std::size_t N> class DeadlineMotionExecutor {
public:
  DeadlineMotionExecutor(FixedJointCommand<N> initial, std::shared_ptr<const IExecutionSchedule> schedule,
                         ClockId clock, ExecutionTimingPolicy policy)
      : execution_schedule_(std::move(schedule)), clock_id_(clock), timing_policy_(policy),
        initial_command_(initial) {
    if (!execution_schedule_ || !clock || execution_schedule_->initial_state().q.size() != N ||
        initial.tick != 0)
      throw std::invalid_argument("Invalid execution schedule");
    for (double x : {policy.max_early_s, policy.max_late_s, policy.max_clock_uncertainty_s,
                     policy.dispatch_latency_bound_s})
      if (!std::isfinite(x) || x < 0)
        throw std::invalid_argument("Invalid timing policy");
    const double window = policy.max_early_s + policy.max_late_s;
    if (window <= 0 || window >= execution_schedule_->cycle_s() ||
        2 * policy.max_clock_uncertainty_s + policy.dispatch_latency_bound_s > window)
      throw std::invalid_argument("Overlapping timing windows");
    joint_limits_ = execution_schedule_->joint_limits();
    execution_epoch_s_ = execution_schedule_->initial_state().timestamp_s;
    cycle_period_s_ = execution_schedule_->cycle_s();
    terminal_tick_ = execution_schedule_->total_ticks();
    rest_boundary_ticks_ = execution_schedule_->boundary_ticks();
  }
  ExecutionFault fault() const noexcept { return fault_; }
  template <std::size_t Capacity>
  ExecutionStatus poll(SpscSetpointBuffer<N, Capacity> &ring, ClockTime now,
                       FixedJointCommand<N> &out) noexcept {
    return poll_impl(ring, now, out, [now] { return now; }, false);
  }
  template <std::size_t Capacity, class Clock>
  ExecutionStatus poll_clock(SpscSetpointBuffer<N, Capacity> &ring, Clock &clock,
                             FixedJointCommand<N> &out) noexcept {
    return poll_impl(ring, clock.now(), out, [&clock] { return clock.now(); }, true);
  }

private:
  bool clock_valid(ClockTime t) noexcept {
    if (!std::isfinite(t.seconds) || !std::isfinite(t.uncertainty_s) || t.uncertainty_s < 0 ||
        t.clock != clock_id_) {
      fault_ = ExecutionFault::InvalidClock;
      return false;
    }
    if (t.uncertainty_s > timing_policy_.max_clock_uncertainty_s) {
      fault_ = ExecutionFault::ExcessUncertainty;
      return false;
    }
    if (last_observed_time_s_ && t.seconds < *last_observed_time_s_) {
      fault_ = ExecutionFault::ClockRegression;
      return false;
    }
    last_observed_time_s_ = t.seconds;
    return true;
  }
  template <std::size_t Capacity, class Observe>
  ExecutionStatus poll_impl(SpscSetpointBuffer<N, Capacity> &ring, ClockTime now, FixedJointCommand<N> &out,
                            Observe observe, bool second) noexcept {
    if (fault_ != ExecutionFault::None)
      return ExecutionStatus::Error;
    if (finished_)
      return ExecutionStatus::Inactive;
    if (!clock_valid(now))
      return ExecutionStatus::Error;
    const double due = execution_epoch_s_ + schedule_detail::tick_time(next_tick_, cycle_period_s_);
    const auto window = [&](ClockTime t) {
      return static_cast<long double>(t.seconds) - t.uncertainty_s >=
                 static_cast<long double>(due) - timing_policy_.max_early_s &&
             static_cast<long double>(t.seconds) + t.uncertainty_s +
                     timing_policy_.dispatch_latency_bound_s <=
                 static_cast<long double>(due) + timing_policy_.max_late_s;
    };
    if (static_cast<long double>(now.seconds) + now.uncertainty_s + timing_policy_.dispatch_latency_bound_s >
        static_cast<long double>(due) + timing_policy_.max_late_s) {
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
    bool valid = buffer_detail::finite(candidate) && candidate.tick == next_tick_ &&
                 candidate.program_time_s == schedule_detail::tick_time(next_tick_, cycle_period_s_) &&
                 candidate.timestamp_s == due && candidate.terminal == (next_tick_ == terminal_tick_);
    for (std::size_t i = 0; i < N; ++i)
      if (candidate.q[i] < joint_limits_.min_position[i] || candidate.q[i] > joint_limits_.max_position[i] ||
          std::abs(candidate.dq[i]) > joint_limits_.max_velocity[i] + 1e-8 ||
          std::abs(candidate.ddq[i]) > joint_limits_.max_acceleration[i] + 1e-8)
        valid = false;
    if (next_tick_ == 0 && candidate != initial_command_)
      valid = false;
    if (next_rest_boundary_index_ < rest_boundary_ticks_.size() &&
        rest_boundary_ticks_[next_rest_boundary_index_] == next_tick_) {
      if (!buffer_detail::rest(candidate))
        valid = false;
      ++next_rest_boundary_index_;
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
    ++next_tick_;
    finished_ = candidate.terminal;
    return finished_ ? ExecutionStatus::Finished : ExecutionStatus::Running;
  }
  std::shared_ptr<const IExecutionSchedule> execution_schedule_;
  ClockId clock_id_;
  ExecutionTimingPolicy timing_policy_;
  FixedJointCommand<N> initial_command_;
  JointLimits joint_limits_;
  double execution_epoch_s_{}, cycle_period_s_{};
  std::uint64_t next_tick_{}, terminal_tick_{};
  std::vector<std::uint64_t> rest_boundary_ticks_;
  std::size_t next_rest_boundary_index_{};
  std::optional<double> last_observed_time_s_;
  ExecutionFault fault_{ExecutionFault::None};
  bool finished_{};
};
} // namespace motion
