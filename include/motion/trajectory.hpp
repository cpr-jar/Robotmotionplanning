#pragma once
#include "core.hpp"
#include <limits>
#include <memory>
#include <utility>

namespace motion {
// A trajectory is immutable once published to the realtime reader.
// Implementations must be allocation-free in sample(), or document otherwise.
class ITrajectory {
public:
  virtual ~ITrajectory() = default;
  virtual double duration() const noexcept = 0;
  virtual JointState sample(double time_s) const = 0;
};
using TrajectoryPtr = std::shared_ptr<const ITrajectory>;

enum class PlanStatus { Ready, InvalidInput, Unsupported, Infeasible };
struct PlanResult {
  PlanStatus status{PlanStatus::InvalidInput};
  TrajectoryPtr trajectory;
  std::string diagnostic;
  explicit operator bool() const noexcept { return status == PlanStatus::Ready && trajectory != nullptr; }
};
class IJointTrajectoryPlanner {
public:
  virtual ~IJointTrajectoryPlanner() = default;
  virtual PlanResult plan(const JointState &initial, const JointTarget &target) const = 0;
};
struct CartesianRequest {
  Pose start, goal;
  CartesianLimits limits;
};
class ICartesianPathPlanner {
public:
  virtual ~ICartesianPathPlanner() = default;
  virtual PlanResult plan(const CartesianRequest &request, const JointState &initial) const = 0;
};

// Closed-form rest-to-rest minimum-jerk polynomial. This is NOT a general
// online replanner: nonzero boundary velocity/acceleration is explicitly rejected.
// For a linear joint-space path q=q0+(q1-q0)*s, analytic global bounds:
// |s'| <= 15/8 / T, |s''| <= 10/sqrt(3) / T^2, |s'''| <= 60/T^3.
class QuinticRestTrajectory final : public ITrajectory {
public:
  QuinticRestTrajectory(Vector start, Vector destination, double duration_s, double start_time_s = 0)
      : q0_(std::move(start)), delta_(std::move(destination)), T_(duration_s), epoch_(start_time_s) {
    if (q0_.empty() || q0_.size() != delta_.size() || !std::isfinite(T_) || T_ <= 0)
      throw std::invalid_argument("Invalid quintic trajectory");
    for (std::size_t i = 0; i < q0_.size(); ++i) {
      if (!std::isfinite(q0_[i]) || !std::isfinite(delta_[i]))
        throw std::invalid_argument("Nonfinite trajectory");
      delta_[i] -= q0_[i];
    }
  }
  double duration() const noexcept override { return T_; }
  JointState sample(double time_s) const override {
    if (!std::isfinite(time_s))
      throw std::invalid_argument("Nonfinite time");
    const double u = std::clamp(time_s / T_, 0.0, 1.0);
    const double u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
    const double s = 10 * u3 - 15 * u4 + 6 * u5;
    const double ds = (30 * u2 - 60 * u3 + 30 * u4) / T_;
    const double dds = (60 * u - 180 * u2 + 120 * u3) / (T_ * T_);
    JointState result;
    result.timestamp_s = epoch_ + std::clamp(time_s, 0.0, T_);
    result.q.resize(q0_.size());
    result.dq.resize(q0_.size());
    result.ddq.resize(q0_.size());
    for (std::size_t i = 0; i < q0_.size(); ++i) {
      result.q[i] = q0_[i] + delta_[i] * s;
      result.dq[i] = delta_[i] * ds;
      result.ddq[i] = delta_[i] * dds;
    }
    return result;
  }
  double peak_jerk(std::size_t axis) const { return std::abs(delta_.at(axis)) * 60 / (T_ * T_ * T_); }

private:
  Vector q0_, delta_;
  double T_, epoch_;
};
inline bool is_rest(const JointState &s, double tolerance = 1e-10) {
  for (double x : s.dq)
    if (!std::isfinite(x) || std::abs(x) > tolerance)
      return false;
  for (double x : s.ddq)
    if (!std::isfinite(x) || std::abs(x) > tolerance)
      return false;
  return true;
}
inline bool is_rest(const JointTarget &s, double tolerance = 1e-10) {
  for (double x : s.dq)
    if (!std::isfinite(x) || std::abs(x) > tolerance)
      return false;
  for (double x : s.ddq)
    if (!std::isfinite(x) || std::abs(x) > tolerance)
      return false;
  return true;
}

// Conservative, analytic bound on an entire rest-to-rest straight joint segment.
class QuinticJointPlanner final : public IJointTrajectoryPlanner {
public:
  explicit QuinticJointPlanner(JointLimits limits) : limits_(std::move(limits)) {
    check_limits(limits_, limits_.min_position.size());
  }
  PlanResult plan(const JointState &initial, const JointTarget &target) const override {
    const auto n = limits_.min_position.size();
    if (!std::isfinite(initial.timestamp_s))
      return {PlanStatus::InvalidInput, {}, "Nonfinite epoch"};
    if (initial.q.size() != n || target.q.size() != n || target.dq.size() != n || target.ddq.size() != n ||
        initial.dq.size() != n || initial.ddq.size() != n)
      return {PlanStatus::InvalidInput, {}, "Dimension mismatch"};
    if (!within_joint_limits(initial, limits_) || !is_rest(initial, 0) || !is_rest(target, 0))
      return {PlanStatus::Unsupported, {}, "Only rest-to-rest motion is implemented"};
    double T = 0.001;
    for (std::size_t i = 0; i < n; ++i) {
      if (!std::isfinite(target.q[i]) || target.q[i] < limits_.min_position[i] ||
          target.q[i] > limits_.max_position[i])
        return {PlanStatus::Infeasible, {}, "Target violates position limit"};
      const auto d = std::abs(target.q[i] - initial.q[i]);
      T = std::max(T, 1.875 * d / limits_.max_velocity[i]);
      T = std::max(T, std::sqrt((10 / std::sqrt(3.0)) * d / limits_.max_acceleration[i]));
      T = std::max(T, std::cbrt(60 * d / limits_.max_jerk[i]));
    }
    if (!std::isfinite(T))
      return {PlanStatus::Infeasible, {}, "Duration overflow"};
    // Small margin for floating point rounding.
    T *= 1.000001;
    return {PlanStatus::Ready,
            std::make_shared<QuinticRestTrajectory>(initial.q, target.q, T, initial.timestamp_s),
            "Analytic full-interval v/a/j bounds"};
  }

private:
  JointLimits limits_;
};

// Implemented for a fixed-orientation XYZ gantry. Not valid for serial/Delta arms.
// Constant-orientation TCP straight line: derivatives are identical to the XYZ joints,
// so the joint planner's analytic bounds also bound Cartesian linear v/a/jerk.
class GantryLinearPlanner final : public ICartesianPathPlanner {
public:
  GantryLinearPlanner(JointLimits limits, const CartesianGantry &kinematics)
      : joints_(std::move(limits)), kinematics_(kinematics) {}
  PlanResult plan(const CartesianRequest &r, const JointState &initial) const override {
    if (!std::isfinite(r.limits.max_linear_velocity) || !std::isfinite(r.limits.max_linear_acceleration) ||
        !std::isfinite(r.limits.max_linear_jerk) || r.limits.max_linear_velocity <= 0 ||
        r.limits.max_linear_acceleration <= 0 || r.limits.max_linear_jerk <= 0)
      return {PlanStatus::InvalidInput, {}, "Invalid Cartesian limits"};
    if (initial.q.size() != 3)
      return {PlanStatus::InvalidInput, {}, "XYZ gantry required"};
    auto initial_pose = kinematics_.fk(initial.q);
    for (std::size_t i = 0; i < 3; ++i)
      if (!std::isfinite(r.start.position[i]) || !std::isfinite(r.goal.position[i]) ||
          std::abs(initial_pose.position[i] - r.start.position[i]) > 1e-8)
        return {PlanStatus::InvalidInput, {}, "Starting pose mismatch"};
    const auto target = kinematics_.ik(r.goal, initial.q);
    const auto origin = kinematics_.ik(r.start, initial.q);
    if (!target || !origin)
      return {PlanStatus::Unsupported, {}, "Rotated TCP unsupported by XYZ gantry"};
    auto p = joints_.plan(initial, {*target, Vector(3, 0), Vector(3, 0)});
    if (!p)
      return p;
    double distance = 0;
    for (std::size_t i = 0; i < 3; ++i)
      distance += std::pow((*target)[i] - initial.q[i], 2);
    distance = std::sqrt(distance);
    const double T = p.trajectory->duration();
    const double Tcart =
        std::max({1.875 * distance / r.limits.max_linear_velocity,
                  std::sqrt((10 / std::sqrt(3.0)) * distance / r.limits.max_linear_acceleration),
                  std::cbrt(60 * distance / r.limits.max_linear_jerk)});
    if (!std::isfinite(Tcart))
      return {PlanStatus::Infeasible, {}, "Duration overflow"};
    return {PlanStatus::Ready,
            std::make_shared<QuinticRestTrajectory>(initial.q, *target, std::max(T, Tcart) * 1.000001,
                                                    initial.timestamp_s),
            "XYZ gantry only: analytic Cartesian and joint bounds"};
  }

private:
  QuinticJointPlanner joints_;
  const CartesianGantry &kinematics_;
};
} // namespace motion
