#pragma once
#include "motion/execution/execution_schedule.hpp"
#include "motion/program/joint_program.hpp"
namespace motion {
class CycleAlignedProgram;
struct CycleGridResult {
  PlanStatus status{PlanStatus::InvalidInput};
  std::shared_ptr<const CycleAlignedProgram> program;
  std::string diagnostic;
  explicit operator bool() const { return status == PlanStatus::Ready && bool(program); }
};
CycleGridResult align_program_cycles(std::shared_ptr<const IJointProgram>, double);
class CycleAlignedProgram final : public IJointProgram, public IExecutionSchedule {
public:
  double duration() const noexcept override { return ends_.back(); }
  double cycle_s() const noexcept override { return cycle_; }
  std::uint64_t total_ticks() const noexcept override { return ticks_.back(); }
  double tick_time(std::uint64_t tick) const noexcept override {
    return tick <= total_ticks() ? schedule_detail::tick_time(tick, cycle_) : NAN;
  }
  const std::vector<std::uint64_t> &boundary_ticks() const noexcept override { return ticks_; }
  std::size_t segment_count() const noexcept override { return ends_.size(); }
  double segment_end(std::size_t i) const override { return ends_.at(i); }
  double segment_duration(std::size_t i) const override { return ends_.at(i) - (i ? ends_[i - 1] : 0); }
  double time_scale(std::size_t i) const { return scales_.at(i); }
  const JointLimits &joint_limits() const noexcept override { return source_->joint_limits(); }
  const JointState &initial_state() const noexcept override { return source_->initial_state(); }
  JointState sample_segment(std::size_t i, double t) const override {
    if (!std::isfinite(t))
      throw std::invalid_argument("Nonfinite grid time");
    const double span = segment_duration(i);
    t = std::clamp(t, 0., span);
    auto out = source_->sample_segment(i, t == span ? source_->segment_duration(i)
                                                    : (t / span) * source_->segment_duration(i));
    const double s = scales_.at(i);
    for (auto &v : out.dq)
      v *= s;
    for (auto &a : out.ddq)
      a *= s * s;
    out.timestamp_s = initial_state().timestamp_s + (i ? ends_[i - 1] : 0) + t;
    return out;
  }
  JointState sample(double t) const override {
    if (!std::isfinite(t) || t < 0 || t > duration())
      throw std::invalid_argument("Grid time outside interval");
    const auto it = std::upper_bound(ends_.begin(), ends_.end(), t);
    const auto i = it == ends_.end() ? ends_.size() - 1 : std::size_t(it - ends_.begin());
    return sample_segment(i, t - (i ? ends_[i - 1] : 0));
  }
  JointState sample_tick(std::uint64_t tick) const override {
    if (tick > total_ticks())
      throw std::invalid_argument("Tick outside program");
    const auto it = std::upper_bound(ticks_.begin(), ticks_.end(), tick);
    const auto i = it == ticks_.end() ? ticks_.size() - 1 : std::size_t(it - ticks_.begin());
    auto out = sample_segment(i, tick_time(tick) - (i ? ends_[i - 1] : 0));
    out.timestamp_s = initial_state().timestamp_s + tick_time(tick);
    return out;
  }

private:
  friend CycleGridResult align_program_cycles(std::shared_ptr<const IJointProgram>, double);
  std::shared_ptr<const IJointProgram> source_;
  double cycle_{};
  Vector ends_, scales_;
  std::vector<std::uint64_t> ticks_;
};
inline CycleGridResult align_program_cycles(std::shared_ptr<const IJointProgram> source, double cycle) {
  const auto fail = [](const char *d) { return CycleGridResult{PlanStatus::InvalidInput, {}, d}; };
  if (!source || !source->segment_count() || !std::isfinite(cycle) || cycle <= 0)
    return fail("Invalid execution source/cycle");
  try {
    const auto initial = source->initial_state();
    if (!within_joint_limits(initial, source->joint_limits(), 0) || !is_rest(initial, 0))
      return fail("Invalid initial rest");
    auto grid = std::shared_ptr<CycleAlignedProgram>(new CycleAlignedProgram);
    grid->source_ = source;
    grid->cycle_ = cycle;
    double begin = 0;
    std::uint64_t total = 0;
    auto previous = initial;
    for (std::size_t i = 0; i < source->segment_count(); ++i) {
      const double duration = source->segment_duration(i);
      if (!std::isfinite(duration) || duration <= 0)
        return fail("Invalid segment duration");
      const long double count = std::ceil(static_cast<long double>(duration) / cycle);
      if (!std::isfinite(count) || count > schedule_detail::max_ticks - total)
        return fail("Tick budget exceeded");
      auto end_tick = total + std::uint64_t(count);
      double end = schedule_detail::tick_time(end_tick, cycle);
      for (int attempt = 0; end - begin < duration && attempt < 8; ++attempt) {
        ++end_tick;
        end = schedule_detail::tick_time(end_tick, cycle);
      }
      if (end_tick > schedule_detail::max_ticks || end - begin < duration)
        return fail("Cycle alignment resolution failure");
      auto first = source->sample_segment(i, 0), last = source->sample_segment(i, duration);
      if (!within_joint_limits(first, source->joint_limits(), 0) ||
          !within_joint_limits(last, source->joint_limits(), 0) || !is_rest(first, 0) || !is_rest(last, 0))
        return fail("Execution joins must be rest");
      for (std::size_t k = 0; k < initial.q.size(); ++k)
        if (std::abs(first.q[k] - previous.q[k]) > 1e-9)
          return fail("Discontinuous program join");
      grid->ends_.push_back(end);
      grid->ticks_.push_back(end_tick);
      grid->scales_.push_back(duration / (end - begin));
      previous = last;
      total = end_tick;
      begin = end;
    }
    if (!schedule_detail::valid_clock(initial.timestamp_s, cycle, total))
      return fail("Insufficient grid clock resolution");
    return {PlanStatus::Ready, std::move(grid), "Non-shortened exact-stop cycle grid"};
  } catch (const std::exception &e) {
    return {PlanStatus::InvalidInput, {}, e.what()};
  }
}
} // namespace motion
