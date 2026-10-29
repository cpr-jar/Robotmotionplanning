#pragma once
#ifndef MOTION_WITH_RUCKIG
#error "Enable MOTION_WITH_RUCKIG for OnlineJointPlanner"
#endif
#include "jerk_phase.hpp"
#include "online.hpp"
#include <ruckig/ruckig.hpp>
namespace motion {
// Synchronous non-RT planning/sampling. Joint-only; no Cartesian geometry claim.
class OnlineJointPlanner final : public IOnlineJointGenerator {
public:
  OnlineJointPlanner(JointLimits limits, double cycle)
      : limits_(std::move(limits)), cycle_(cycle), solver_(limits_.max_velocity.size(), cycle) {
    check_limits(limits_, limits_.max_velocity.size());
    if (!std::isfinite(cycle) || cycle <= 0)
      throw std::invalid_argument("Invalid online cycle");
  }
  StepStatus reset(const JointState &s) override {
    try {
      if (!std::isfinite(s.timestamp_s) || !within_joint_limits(s, limits_, 0))
        return StepStatus::InvalidInput;
    } catch (...) {
      return StepStatus::InvalidInput;
    }
    state_ = s;
    origin_ = s;
    trajectory_.reset();
    elapsed_ = 0;
    initialized_ = true;
    return StepStatus::Finished;
  }
  StepStatus retarget(const JointTarget &target) override {
    const auto n = limits_.max_velocity.size();
    if (!initialized_ || target.q.size() != n || target.dq.size() != n || target.ddq.size() != n)
      return StepStatus::InvalidInput;
    try {
      if (!within_joint_limits({target.q, target.dq, target.ddq, state_.timestamp_s}, limits_, 0))
        return StepStatus::InvalidInput;
      ruckig::InputParameter<ruckig::DynamicDOFs> in(n);
      in.current_position = state_.q;
      in.current_velocity = state_.dq;
      in.current_acceleration = state_.ddq;
      in.target_position = target.q;
      in.target_velocity = target.dq;
      in.target_acceleration = target.ddq;
      in.max_velocity = limits_.max_velocity;
      in.max_acceleration = limits_.max_acceleration;
      in.max_jerk = limits_.max_jerk;
      in.synchronization = ruckig::Synchronization::Time;
      auto candidate = std::make_shared<ruckig::Trajectory<ruckig::DynamicDOFs>>(n);
      if (int(solver_.calculate(in, *candidate)) < 0)
        return StepStatus::Error;
      auto extrema = candidate->get_position_extrema();
      for (std::size_t i = 0; i < n; ++i)
        if (extrema[i].min < limits_.min_position[i] || extrema[i].max > limits_.max_position[i])
          return StepStatus::Error;
      const auto profiles = candidate->get_profiles();
      if (profiles.size() != 1)
        return StepStatus::Error;
      for (std::size_t i = 0; i < n; ++i) {
        const auto &p = profiles[0][i];
        phase_detail::Bounds bounds;
        bool good = true;
        const auto check = [&](double t, double j, double q, double v, double a) {
          if (t > 0 && !phase_detail::check({0, t, q, v, a, j}, limits_.min_position[i],
                                            limits_.max_position[i], limits_.max_velocity[i],
                                            limits_.max_acceleration[i], limits_.max_jerk[i], bounds))
            good = false;
        };
        if (p.brake.duration > 0)
          for (int k = 0; k < 2; ++k)
            check(p.brake.t[k], p.brake.j[k], p.brake.p[k], p.brake.v[k], p.brake.a[k]);
        for (int k = 0; k < 7; ++k)
          check(p.t[k], p.j[k], p.p[k], p.v[k], p.a[k]);
        if (!good)
          return StepStatus::Error;
      }
      if (!std::isfinite(candidate->get_duration()) ||
          !std::isfinite(state_.timestamp_s + candidate->get_duration()))
        return StepStatus::Error;
      trajectory_ = std::move(candidate);
      origin_ = state_;
      elapsed_ = 0;
      return StepStatus::Running;
    } catch (...) {
      return StepStatus::Error;
    }
  }
  void set_target(const JointTarget &target) {
    if (retarget(target) != StepStatus::Running)
      throw std::invalid_argument("Online target/candidate rejected");
  }
  StepStatus step(JointState &out) override {
    if (!initialized_ || !trajectory_)
      return StepStatus::InvalidInput;
    const double time = std::min(elapsed_ + cycle_, trajectory_->get_duration());
    JointState candidate{Vector(limits_.max_velocity.size()), Vector(limits_.max_velocity.size()),
                         Vector(limits_.max_velocity.size()), origin_.timestamp_s + time};
    trajectory_->at_time(time, candidate.q, candidate.dq, candidate.ddq);
    if (!within_joint_limits(candidate, limits_))
      return StepStatus::Error;
    state_ = candidate;
    out = std::move(candidate);
    elapsed_ = time;
    const bool finished = time == trajectory_->get_duration();
    if (finished)
      trajectory_.reset();
    return finished ? StepStatus::Finished : StepStatus::Running;
  }

private:
  JointLimits limits_;
  double cycle_, elapsed_{};
  ruckig::Ruckig<ruckig::DynamicDOFs> solver_;
  std::shared_ptr<ruckig::Trajectory<ruckig::DynamicDOFs>> trajectory_;
  JointState state_, origin_;
  bool initialized_{};
};
} // namespace motion
