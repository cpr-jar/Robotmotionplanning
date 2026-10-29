#pragma once
#include "execution_schedule.hpp"
#include "joint_stop.hpp"
namespace motion {
class JointStopSchedule final : public IExecutionSchedule {
public:
  JointStopSchedule(std::shared_ptr<const JointStopTrajectory> stop, double cycle)
      : stop_(std::move(stop)), cycle_(cycle) {
    if (!stop_ || !std::isfinite(cycle) || cycle <= 0)
      throw std::invalid_argument("Invalid stop schedule");
    const long double ticks = std::ceil(static_cast<long double>(stop_->duration()) / cycle);
    if (ticks > schedule_detail::max_ticks)
      throw std::invalid_argument("Stop tick overflow");
    ticks_ = std::uint64_t(std::max(1.L, ticks));
    while (tick_time(ticks_) < stop_->duration())
      ++ticks_;
    if (!schedule_detail::valid_clock(initial_state().timestamp_s, cycle, ticks_))
      throw std::invalid_argument("Stop schedule clock precision");
    boundaries_ = {ticks_};
  }
  double duration() const noexcept override { return tick_time(ticks_); }
  double cycle_s() const noexcept override { return cycle_; }
  std::uint64_t total_ticks() const noexcept override { return ticks_; }
  double tick_time(std::uint64_t tick) const noexcept override {
    return tick <= ticks_ ? schedule_detail::tick_time(tick, cycle_) : NAN;
  }
  const std::vector<std::uint64_t> &boundary_ticks() const noexcept override { return boundaries_; }
  const JointState &initial_state() const noexcept override { return stop_->initial_state(); }
  const JointLimits &joint_limits() const noexcept override { return stop_->joint_limits(); }
  const auto &stop() const { return stop_; }
  JointState sample(double t) const override {
    if (!std::isfinite(t) || t < 0 || t > duration())
      throw std::invalid_argument("Stop schedule time outside interval");
    auto s = stop_->sample(std::min(t, stop_->duration()));
    s.timestamp_s = initial_state().timestamp_s + t;
    return s;
  }
  JointState sample_tick(std::uint64_t tick) const override {
    if (tick > ticks_)
      throw std::invalid_argument("Stop tick outside interval");
    return sample(tick_time(tick));
  }

private:
  std::shared_ptr<const JointStopTrajectory> stop_;
  double cycle_;
  std::uint64_t ticks_;
  std::vector<std::uint64_t> boundaries_;
};
struct StopScheduleResult {
  StopPlanStatus status{};
  std::shared_ptr<const JointStopSchedule> program;
  std::string diagnostic;
  explicit operator bool() const { return bool(program); }
};
inline StopScheduleResult schedule_joint_stop(std::shared_ptr<const JointStopTrajectory> stop, double cycle) {
  try {
    if (!stop)
      return {StopPlanStatus::InvalidInput, {}, "Missing stop"};
    const auto status = stop->duration() == 0 ? StopPlanStatus::AlreadyStopped : StopPlanStatus::Ready;
    return {status, std::make_shared<JointStopSchedule>(std::move(stop), cycle),
            "Original stop time law plus rest padding"};
  } catch (const std::exception &e) {
    return {StopPlanStatus::InvalidInput, {}, e.what()};
  }
}
class PolynomialStopSchedule final : public IExecutionSchedule {
public:
  PolynomialStopSchedule(std::shared_ptr<const BoundaryQuinticTrajectory> prefix,
                         std::shared_ptr<const JointStopTrajectory> stop, double cycle)
      : prefix_(std::move(prefix)), stop_(std::move(stop)), cycle_(cycle) {
    const long double count =
        std::ceil((static_cast<long double>(prefix_->duration()) + stop_->duration()) / cycle);
    if (!std::isfinite(cycle) || cycle <= 0 || count > schedule_detail::max_ticks || count < 1)
      throw std::invalid_argument("Invalid following schedule clock");
    ticks_ = std::uint64_t(count);
    for (int retry = 0; tick_time(ticks_) - prefix_->duration() < stop_->duration() && retry < 8; ++retry)
      ++ticks_;
    if (tick_time(ticks_) - prefix_->duration() < stop_->duration() ||
        !schedule_detail::valid_clock(initial_state().timestamp_s, cycle, ticks_))
      throw std::invalid_argument("Following schedule precision");
    boundaries_ = {ticks_};
  }
  double duration() const noexcept override { return tick_time(ticks_); }
  double cycle_s() const noexcept override { return cycle_; }
  std::uint64_t total_ticks() const noexcept override { return ticks_; }
  double tick_time(std::uint64_t tick) const noexcept override {
    return tick <= ticks_ ? schedule_detail::tick_time(tick, cycle_) : NAN;
  }
  const std::vector<std::uint64_t> &boundary_ticks() const noexcept override { return boundaries_; }
  const JointState &initial_state() const noexcept override { return prefix_->initial_state(); }
  const JointLimits &joint_limits() const noexcept override { return prefix_->joint_limits(); }
  const auto &fallback() const { return stop_; }
  double tracking_duration() const { return prefix_->duration(); }
  JointState sample(double t) const override {
    if (!std::isfinite(t) || t < 0 || t > duration())
      throw std::invalid_argument("Following time outside interval");
    auto s = t <= prefix_->duration() ? prefix_->sample(t)
                                      : stop_->sample(std::min(t - prefix_->duration(), stop_->duration()));
    s.timestamp_s = initial_state().timestamp_s + t;
    return s;
  }
  JointState sample_tick(std::uint64_t tick) const override {
    if (tick > ticks_)
      throw std::invalid_argument("Following tick outside interval");
    return sample(tick_time(tick));
  }

private:
  std::shared_ptr<const BoundaryQuinticTrajectory> prefix_;
  std::shared_ptr<const JointStopTrajectory> stop_;
  double cycle_;
  std::uint64_t ticks_;
  std::vector<std::uint64_t> boundaries_;
};
struct PolynomialStopResult {
  std::shared_ptr<const PolynomialStopSchedule> program;
  std::string diagnostic;
  explicit operator bool() const { return bool(program); }
};
inline PolynomialStopResult
make_polynomial_stop_schedule(std::shared_ptr<const BoundaryQuinticTrajectory> prefix, double cycle) {
  if (!prefix)
    return {{}, "Missing polynomial"};
  auto stopped = JointStopPlanner(prefix->joint_limits()).plan(prefix->terminal_state());
  if (!stopped)
    return {{}, stopped.diagnostic};
  try {
    return {std::make_shared<PolynomialStopSchedule>(std::move(prefix), stopped.trajectory, cycle),
            "Moving polynomial plus admitted fallback"};
  } catch (const std::exception &e) {
    return {{}, e.what()};
  }
}
class PreparedStopTransition final : public IExecutionSchedule {
public:
  PreparedStopTransition(std::shared_ptr<const IExecutionSchedule> source, std::uint64_t activation,
                         std::shared_ptr<const JointStopTrajectory> stop)
      : source_(std::move(source)), activation_(activation), stop_(std::move(stop)) {
    const long double count = std::ceil(static_cast<long double>(stop_->duration()) / cycle_s());
    if (count > schedule_detail::max_ticks - activation_)
      throw std::invalid_argument("Transition tick budget");
    ticks_ = activation_ + std::uint64_t(std::max(1.L, count));
    for (int retry = 0; tick_time(ticks_) - tick_time(activation_) < stop_->duration() && retry < 8; ++retry)
      ++ticks_;
    if (tick_time(ticks_) - tick_time(activation_) < stop_->duration() ||
        !schedule_detail::valid_clock(initial_state().timestamp_s, cycle_s(), ticks_))
      throw std::invalid_argument("Transition clock precision");
    for (auto tick : source_->boundary_ticks())
      if (tick < activation_)
        boundaries_.push_back(tick);
    boundaries_.push_back(ticks_);
  }
  double duration() const noexcept override { return tick_time(ticks_); }
  double cycle_s() const noexcept override { return source_->cycle_s(); }
  std::uint64_t total_ticks() const noexcept override { return ticks_; }
  double tick_time(std::uint64_t tick) const noexcept override {
    return tick <= ticks_ ? schedule_detail::tick_time(tick, cycle_s()) : NAN;
  }
  const std::vector<std::uint64_t> &boundary_ticks() const noexcept override { return boundaries_; }
  const JointState &initial_state() const noexcept override { return source_->initial_state(); }
  const JointLimits &joint_limits() const noexcept override { return source_->joint_limits(); }
  JointState sample(double t) const override {
    if (!std::isfinite(t) || t < 0 || t > duration())
      throw std::invalid_argument("Transition time outside interval");
    if (t <= tick_time(activation_))
      return source_->sample(t);
    auto s = stop_->sample(std::min(t - tick_time(activation_), stop_->duration()));
    s.timestamp_s = initial_state().timestamp_s + t;
    return s;
  }
  JointState sample_tick(std::uint64_t tick) const override {
    if (tick > ticks_)
      throw std::invalid_argument("Transition tick outside interval");
    return tick <= activation_ ? source_->sample_tick(tick) : sample(tick_time(tick));
  }

private:
  std::shared_ptr<const IExecutionSchedule> source_;
  std::uint64_t activation_, ticks_;
  std::shared_ptr<const JointStopTrajectory> stop_;
  std::vector<std::uint64_t> boundaries_;
};
struct StopTransitionResult {
  StopPlanStatus status{};
  std::shared_ptr<const PreparedStopTransition> program;
  std::string diagnostic;
  explicit operator bool() const { return bool(program); }
};
inline StopTransitionResult prepare_stop_transition(std::shared_ptr<const IExecutionSchedule> source,
                                                    std::uint64_t activation,
                                                    std::shared_ptr<const JointStopTrajectory> stop) {
  try {
    if (!source || !stop || activation > source->total_ticks())
      throw std::invalid_argument("Invalid transition input");
    const auto s = source->sample_tick(activation);
    const auto &a = stop->initial_state();
    const auto &l = source->joint_limits();
    const auto &r = stop->joint_limits();
    if (s.q != a.q || s.dq != a.dq || s.ddq != a.ddq || s.timestamp_s != a.timestamp_s ||
        l.min_position != r.min_position || l.max_position != r.max_position ||
        l.max_velocity != r.max_velocity || l.max_acceleration != r.max_acceleration ||
        l.max_jerk != r.max_jerk)
      throw std::invalid_argument("Transition state/time/limits mismatch");
    return {StopPlanStatus::Ready,
            std::make_shared<PreparedStopTransition>(std::move(source), activation, std::move(stop)),
            "Prepared source prefix and unchanged stop"};
  } catch (const std::exception &e) {
    return {StopPlanStatus::InvalidInput, {}, e.what()};
  }
}
inline StopTransitionResult prepare_stop_transition(std::shared_ptr<const IExecutionSchedule> source,
                                                    std::uint64_t activation) {
  if (!source || activation > source->total_ticks())
    return {StopPlanStatus::InvalidInput, {}, "Invalid transition input"};
  const auto result = JointStopPlanner(source->joint_limits()).plan(source->sample_tick(activation));
  if (!result)
    return {result.status, {}, result.diagnostic};
  return prepare_stop_transition(std::move(source), activation, result.trajectory);
}
} // namespace motion
