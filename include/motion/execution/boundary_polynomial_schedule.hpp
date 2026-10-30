#pragma once
#include "motion/contracts/execution_schedule.hpp"
#include "motion/planning/boundary_quintic.hpp"
namespace motion {
// A checked polynomial with a rest endpoint; preserve its time law and pad only rest.
class BoundaryPolynomialSchedule final : public IExecutionSchedule {
public:
  BoundaryPolynomialSchedule(std::shared_ptr<const BoundaryQuinticTrajectory> p, double cycle)
      : polynomial_(std::move(p)), cycle_(cycle) {
    if (!polynomial_ || !is_rest(polynomial_->terminal_state(), 0) || !std::isfinite(cycle) || cycle <= 0)
      throw std::invalid_argument("Polynomial schedule needs terminal rest");
    const long double ticks = std::ceil(static_cast<long double>(polynomial_->duration()) / cycle);
    if (ticks > schedule_detail::max_ticks)
      throw std::invalid_argument("Schedule tick budget");
    ticks_ = std::uint64_t(std::max(1.L, ticks));
    while (tick_time(ticks_) < polynomial_->duration())
      ++ticks_;
    if (!schedule_detail::valid_clock(initial_state().timestamp_s, cycle, ticks_))
      throw std::invalid_argument("Schedule clock resolution");
    boundaries_ = {ticks_};
  }
  double duration() const noexcept override { return tick_time(ticks_); }
  double cycle_s() const noexcept override { return cycle_; }
  std::uint64_t total_ticks() const noexcept override { return ticks_; }
  double tick_time(std::uint64_t tick) const noexcept override {
    return tick <= ticks_ ? schedule_detail::tick_time(tick, cycle_) : NAN;
  }
  const std::vector<std::uint64_t> &boundary_ticks() const noexcept override { return boundaries_; }
  const JointState &initial_state() const noexcept override { return polynomial_->initial_state(); }
  const JointLimits &joint_limits() const noexcept override { return polynomial_->joint_limits(); }
  JointState sample(double t) const override {
    if (!std::isfinite(t) || t < 0 || t > duration())
      throw std::invalid_argument("Schedule time outside interval");
    auto r = polynomial_->sample(std::min(t, polynomial_->duration()));
    r.timestamp_s = initial_state().timestamp_s + t;
    return r;
  }
  JointState sample_tick(std::uint64_t tick) const override {
    if (tick > ticks_)
      throw std::invalid_argument("Schedule tick outside interval");
    return sample(tick_time(tick));
  }

private:
  std::shared_ptr<const BoundaryQuinticTrajectory> polynomial_;
  double cycle_;
  std::uint64_t ticks_;
  std::vector<std::uint64_t> boundaries_;
};
} // namespace motion
