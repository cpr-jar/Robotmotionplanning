#pragma once
#ifndef MOTION_WITH_RUCKIG
#error "Enable MOTION_WITH_RUCKIG for joint stopping"
#endif
#include "jerk_phase.hpp"
#include <ruckig/ruckig.hpp>
namespace motion {
enum class StopPlanStatus {
  Ready,
  AlreadyStopped,
  InvalidInput,
  InitialStateOutsideLimits,
  CandidateRejected,
  NumericalFailure
};
using JointStopBounds = phase_detail::Bounds;
struct StopMotionSample {
  JointState joints;
  Vector jerk;
};
class JointStopTrajectory;
struct StopPlanResult {
  StopPlanStatus status{StopPlanStatus::InvalidInput};
  std::shared_ptr<const JointStopTrajectory> trajectory;
  std::optional<std::size_t> failed_axis;
  std::string diagnostic;
  explicit operator bool() const {
    return bool(trajectory) && (status == StopPlanStatus::Ready || status == StopPlanStatus::AlreadyStopped);
  }
};
class JointStopPlanner;
class JointStopTrajectory final : public ITrajectory {
public:
  double duration() const noexcept override { return duration_; }
  const JointState &initial_state() const { return initial_; }
  const JointState &terminal_state() const { return terminal_; }
  const JointLimits &joint_limits() const { return limits_; }
  const auto &bounds() const { return bounds_; }
  const Vector &axis_durations() const { return axis_durations_; }
  JointState sample(double t) const override { return sample_motion(t).joints; }
  StopMotionSample sample_motion(double t) const {
    if (!std::isfinite(t) || t < 0 || t > duration_)
      throw std::invalid_argument("Stop sample outside interval");
    StopMotionSample r{initial_, Vector(initial_.q.size())};
    r.joints.timestamp_s = initial_.timestamp_s + t;
    for (std::size_t i = 0; i < phases_.size(); ++i) {
      bool moving = false;
      for (const auto &p : phases_[i])
        if (static_cast<long double>(t) < p.begin + p.h) {
          const auto s = p.at(std::max(0.L, static_cast<long double>(t) - p.begin));
          r.joints.q[i] = double(s[0]);
          r.joints.dq[i] = double(s[1]);
          r.joints.ddq[i] = double(s[2]);
          r.jerk[i] = double(p.j);
          moving = true;
          break;
        }
      if (!moving) {
        r.joints.q[i] = terminal_.q[i];
        r.joints.dq[i] = 0;
        r.joints.ddq[i] = 0;
      }
    }
    if (t == 0)
      r.joints = initial_;
    if (t == duration_)
      r.joints = terminal_;
    return r;
  }
  using StateBox = std::array<std::pair<double, double>, 4>;
  std::vector<StateBox> enclose_state(double lo, double hi) const {
    if (!std::isfinite(lo) || !std::isfinite(hi) || lo < 0 || lo > hi || hi > duration_)
      throw std::invalid_argument("Stop enclosure outside interval");
    std::vector<StateBox> r(phases_.size());
    for (std::size_t i = 0; i < phases_.size(); ++i) {
      for (auto &v : r[i])
        v = {INFINITY, -INFINITY};
      const auto include = [&](int order, std::initializer_list<long double> values) {
        for (auto x : values) {
          r[i][order].first = std::min(r[i][order].first, std::nextafter(double(x), -INFINITY));
          r[i][order].second = std::max(r[i][order].second, std::nextafter(double(x), INFINITY));
        }
      };
      for (const auto &p : phases_[i]) {
        const long double l = std::max(static_cast<long double>(lo), p.begin),
                          h = std::min(static_cast<long double>(hi), p.begin + p.h);
        if (l > h)
          continue;
        const auto x = p.at(l - p.begin);
        const auto span = h - l;
        include(0, {x[0], x[0] + x[1] * span / 3, x[0] + 2 * x[1] * span / 3 + x[2] * span * span / 6,
                    x[0] + x[1] * span + x[2] * span * span / 2 + p.j * span * span * span / 6});
        include(1, {x[1], x[1] + x[2] * span / 2, x[1] + x[2] * span + p.j * span * span / 2});
        include(2, {x[2], x[2] + p.j * span});
        include(3, {p.j});
      }
      if (hi >= axis_durations_[i]) {
        include(0, {terminal_.q[i]});
        for (int o = 1; o < 4; ++o)
          include(o, {0});
      }
    }
    return r;
  }

private:
  friend class JointStopPlanner;
  JointStopTrajectory() = default;
  JointState initial_, terminal_;
  JointLimits limits_;
  std::vector<std::vector<phase_detail::Phase>> phases_;
  Vector axis_durations_;
  std::vector<JointStopBounds> bounds_;
  double duration_{};
};
class JointStopPlanner {
public:
  explicit JointStopPlanner(JointLimits l) : limits_(std::move(l)) {
    check_limits(limits_, limits_.min_position.size());
  }
  StopPlanResult plan(const JointState &s) const {
    const auto fail = [](StopPlanStatus status, const char *d,
                         std::optional<std::size_t> axis = std::nullopt) {
      return StopPlanResult{status, {}, axis, d};
    };
    const auto n = limits_.min_position.size();
    if (s.q.size() != n || s.dq.size() != n || s.ddq.size() != n || !std::isfinite(s.timestamp_s))
      return fail(StopPlanStatus::InvalidInput, "Invalid stop state");
    for (const auto *v : {&s.q, &s.dq, &s.ddq})
      for (double x : *v)
        if (!std::isfinite(x))
          return fail(StopPlanStatus::InvalidInput, "Nonfinite stop state");
    if (!within_joint_limits(s, limits_, 0))
      return fail(StopPlanStatus::InitialStateOutsideLimits, "Stop starts outside limits");
    try {
      auto candidate = std::shared_ptr<JointStopTrajectory>(new JointStopTrajectory);
      candidate->initial_ = s;
      candidate->terminal_ = s;
      candidate->limits_ = limits_;
      candidate->phases_.resize(n);
      candidate->axis_durations_.resize(n);
      candidate->bounds_.resize(n);
      if (is_rest(s, 0)) {
        for (std::size_t i = 0; i < n; ++i) {
          candidate->bounds_[i].min_position = s.q[i];
          candidate->bounds_[i].max_position = s.q[i];
        }
        return {StopPlanStatus::AlreadyStopped, std::move(candidate), {}, "Already at rest"};
      }
      ruckig::Ruckig<ruckig::DynamicDOFs> solver(n);
      ruckig::InputParameter<ruckig::DynamicDOFs> in(n);
      ruckig::Trajectory<ruckig::DynamicDOFs> trajectory(n);
      in.control_interface = ruckig::ControlInterface::Velocity;
      in.synchronization = ruckig::Synchronization::None;
      in.current_position = s.q;
      in.current_velocity = s.dq;
      in.current_acceleration = s.ddq;
      in.target_velocity = Vector(n);
      in.target_acceleration = Vector(n);
      in.max_velocity = limits_.max_velocity;
      in.max_acceleration = limits_.max_acceleration;
      in.max_jerk = limits_.max_jerk;
      const auto result = solver.calculate(in, trajectory);
      if (int(result) < 0)
        return fail(StopPlanStatus::NumericalFailure, "Ruckig stop calculation failed");
      const auto profiles = trajectory.get_profiles();
      if (profiles.size() != 1)
        return fail(StopPlanStatus::CandidateRejected, "Unexpected stop sections");
      for (std::size_t i = 0; i < n; ++i) {
        const auto &p = profiles[0][i];
        long double t = 0;
        std::array<long double, 3> state{s.q[i], s.dq[i], s.ddq[i]};
        bool good = true;
        const auto append = [&](double h, double j) {
          if (h == 0)
            return;
          if (!std::isfinite(h) || h < 0) {
            good = false;
            return;
          }
          phase_detail::Phase phase{t, h, state[0], state[1], state[2], j};
          if (!phase_detail::check(phase, limits_.min_position[i], limits_.max_position[i],
                                   limits_.max_velocity[i], limits_.max_acceleration[i], limits_.max_jerk[i],
                                   candidate->bounds_[i]))
            good = false;
          candidate->phases_[i].push_back(phase);
          state = phase.at(h);
          t += h;
        };
        if (p.brake.duration > 0)
          for (int k = 0; k < 2; ++k)
            append(p.brake.t[k], p.brake.j[k]);
        for (int k = 0; k < 7; ++k)
          append(p.t[k], p.j[k]);
        if (!good)
          return fail(StopPlanStatus::CandidateRejected, "Stop phase exceeds joint limits", i);
        if (std::abs(state[1]) > 1e-10 * limits_.max_velocity[i] ||
            std::abs(state[2]) > 1e-10 * limits_.max_acceleration[i])
          return fail(StopPlanStatus::NumericalFailure, "Stop endpoint residual", i);
        candidate->terminal_.q[i] = double(state[0]);
        candidate->terminal_.dq[i] = 0;
        candidate->terminal_.ddq[i] = 0;
        candidate->axis_durations_[i] = double(t);
        candidate->duration_ = std::max(candidate->duration_, double(t));
      }
      if (candidate->duration_ <= 0 || !std::isfinite(candidate->duration_))
        return fail(StopPlanStatus::NumericalFailure, "Moving stop has no positive duration");
      candidate->terminal_.timestamp_s = s.timestamp_s + candidate->duration_;
      const double mag = std::max(std::abs(s.timestamp_s), std::abs(candidate->terminal_.timestamp_s));
      if (!std::isfinite(candidate->terminal_.timestamp_s) ||
          candidate->duration_ <= 4 * (std::nextafter(mag, INFINITY) - mag))
        return fail(StopPlanStatus::NumericalFailure, "Stop clock precision");
      return {StopPlanStatus::Ready,
              std::move(candidate),
              {},
              "Independent jerk-phase integration and full extrema admission"};
    } catch (const std::exception &e) {
      return {StopPlanStatus::NumericalFailure, {}, {}, e.what()};
    }
  }

private:
  JointLimits limits_;
};
} // namespace motion
