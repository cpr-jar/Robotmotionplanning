#pragma once
#include "motion/planning/cartesian_path.hpp"
#include "motion/planning/joint_trajectory.hpp"
namespace motion {
struct TimedPathSample {
  JointState joints;
  Vector joint_jerk;
  Pose tcp;
  KinematicDerivatives tcp_derivatives;
};
class PathTimeParameterizer;
class TimedPathTrajectory final : public ITrajectory {
public:
  double duration() const noexcept override { return duration_; }
  const JointPathBounds &path_bounds() const { return bounds_; }
  const auto &bounded_path() const { return path_; }
  JointState sample(double t) const override { return sample_motion(t).joints; }
  TimedPathSample sample_motion(double t) const {
    if (!std::isfinite(t))
      throw std::invalid_argument("Nonfinite path time");
    t = std::clamp(t, 0., duration_);
    const double u = t / duration_, one = 1 - u, s = std::clamp(u * u * u * (10 + u * (-15 + 6 * u)), 0., 1.),
                 v = 30 * u * u * one * one / duration_,
                 a = 60 * u * one * (1 - 2 * u) / (duration_ * duration_),
                 j = (60 - 360 * u + 360 * u * u) / (duration_ * duration_ * duration_);
    const auto p = path_->evaluate(s);
    const auto n = bounds_.lower.size();
    check_point(p, n);
    TimedPathSample out;
    out.joints = {p.q, Vector(n), Vector(n), epoch_ + t};
    out.joint_jerk.resize(n);
    out.tcp = p.tcp.pose;
    for (std::size_t i = 0; i < n; ++i) {
      out.joints.dq[i] = p.q_s[i] * v;
      out.joints.ddq[i] = p.q_ss[i] * v * v + p.q_s[i] * a;
      out.joint_jerk[i] = p.q_sss[i] * v * v * v + 3 * p.q_ss[i] * v * a + p.q_s[i] * j;
    }
    for (int i = 0; i < 3; ++i) {
      out.tcp_derivatives.velocity.data[i] = p.tcp.derivatives[0][i] * v;
      out.tcp_derivatives.acceleration.data[i] =
          p.tcp.derivatives[1][i] * v * v + p.tcp.derivatives[0][i] * a;
      out.tcp_derivatives.jerk.data[i] = p.tcp.derivatives[2][i] * v * v * v +
                                         3 * p.tcp.derivatives[1][i] * v * a + p.tcp.derivatives[0][i] * j;
      out.tcp_derivatives.velocity.data[i + 3] = p.tcp.angular_derivatives[0][i] * v;
      out.tcp_derivatives.acceleration.data[i + 3] =
          p.tcp.angular_derivatives[1][i] * v * v + p.tcp.angular_derivatives[0][i] * a;
      out.tcp_derivatives.jerk.data[i + 3] = p.tcp.angular_derivatives[2][i] * v * v * v +
                                             3 * p.tcp.angular_derivatives[1][i] * v * a +
                                             p.tcp.angular_derivatives[0][i] * j;
    }
    kinematic_detail::require_finite(out.tcp_derivatives);
    if (!kinematic_detail::finite(out.joints.dq, n) || !kinematic_detail::finite(out.joints.ddq, n) ||
        !kinematic_detail::finite(out.joint_jerk, n))
      throw std::invalid_argument("Nonfinite timed path derivatives");
    return out;
  }

private:
  friend class PathTimeParameterizer;
  static void check_point(const JointPathPoint &p, std::size_t n) {
    if (!kinematic_detail::finite(p.q, n) || !kinematic_detail::finite(p.q_s, n) ||
        !kinematic_detail::finite(p.q_ss, n) || !kinematic_detail::finite(p.q_sss, n) ||
        !kinematic_detail::finite_pose(p.tcp.pose))
      throw std::invalid_argument("Invalid path point dimension/value");
    for (const auto &derivative : p.tcp.derivatives)
      path_detail::finite3(derivative);
    for (const auto &derivative : p.tcp.angular_derivatives)
      path_detail::finite3(derivative);
  }
  std::shared_ptr<const IBoundedJointPath> path_;
  JointPathBounds bounds_;
  double duration_{}, epoch_{};
};
class PathTimeParameterizer {
public:
  explicit PathTimeParameterizer(JointLimits l) : limits_(std::move(l)) {
    check_limits(limits_, limits_.min_position.size());
  }
  PlanResult plan(std::shared_ptr<const IBoundedJointPath> path, const JointState &start,
                  const CartesianLimits &caps) const {
    const auto n = limits_.min_position.size();
    if (!path || !kinematic_detail::finite(start.q, n) || !kinematic_detail::finite(start.dq, n) ||
        !kinematic_detail::finite(start.ddq, n) || !is_rest(start, 0) || !std::isfinite(start.timestamp_s))
      return {PlanStatus::InvalidInput, {}, "Invalid path initial state"};
    for (double x : {caps.max_linear_velocity, caps.max_linear_acceleration, caps.max_linear_jerk,
                     caps.max_angular_velocity, caps.max_angular_acceleration, caps.max_angular_jerk})
      if (!std::isfinite(x) || x <= 0)
        return {PlanStatus::InvalidInput, {}, "Invalid TCP caps"};
    try {
      auto b = path->bounds();
      if (!kinematic_detail::finite(b.lower, n) || !kinematic_detail::finite(b.upper, n))
        throw std::invalid_argument("Invalid path bounds dimension/value");
      for (const auto &derivative : b.joint_abs)
        if (!kinematic_detail::finite(derivative, n))
          throw std::invalid_argument("Invalid path derivative bounds dimension/value");
      for (std::size_t i = 0; i < start.q.size(); ++i)
        if (b.lower[i] > b.upper[i])
          throw std::invalid_argument("Reversed path bounds");
      for (std::size_t i = 0; i < start.q.size(); ++i)
        if (b.lower[i] < limits_.min_position[i] || b.upper[i] > limits_.max_position[i])
          return {PlanStatus::Infeasible, {}, "Path joint range exceeds profile"};
      const auto initial = path->evaluate(0);
      TimedPathTrajectory::check_point(initial, n);
      for (std::size_t i = 0; i < start.q.size(); ++i)
        if (std::abs(initial.q[i] - start.q[i]) > 1e-9)
          throw std::invalid_argument("Path state mismatch");
      long double T = .001L;
      const auto constrain = [&](double b1, double b2, double b3, double v, double a, double j) {
        for (double x : {b1, b2, b3})
          if (!std::isfinite(x) || x < 0)
            throw std::invalid_argument("Invalid derivative bounds");
        const long double V = 1.875L, A = 10 / std::sqrt(3.L), J = 60;
        T = std::max(T, b1 * V / v);
        T = std::max(T, std::sqrt((b2 * V * V + b1 * A) / a));
        T = std::max(T, std::cbrt((b3 * V * V * V + 3 * b2 * V * A + b1 * J) / j));
      };
      for (std::size_t i = 0; i < start.q.size(); ++i)
        constrain(b.joint_abs[0].at(i), b.joint_abs[1].at(i), b.joint_abs[2].at(i), limits_.max_velocity[i],
                  limits_.max_acceleration[i], limits_.max_jerk[i]);
      constrain(b.linear_abs[0], b.linear_abs[1], b.linear_abs[2], caps.max_linear_velocity,
                caps.max_linear_acceleration, caps.max_linear_jerk);
      constrain(b.angular_abs[0], b.angular_abs[1], b.angular_abs[2], caps.max_angular_velocity,
                caps.max_angular_acceleration, caps.max_angular_jerk);
      const double duration = double(T * 1.000001L);
      if (!std::isfinite(duration) || !std::isfinite(start.timestamp_s + duration))
        throw std::invalid_argument("Path duration overflow");
      auto r = std::shared_ptr<TimedPathTrajectory>(new TimedPathTrajectory);
      r->path_ = std::move(path);
      r->bounds_ = std::move(b);
      r->duration_ = duration;
      r->epoch_ = start.timestamp_s;
      r->sample(0);
      r->sample(duration);
      return {PlanStatus::Ready, std::move(r), "Full-path derivative envelopes; conservative quintic timing"};
    } catch (const std::invalid_argument &e) {
      return {PlanStatus::InvalidInput, {}, e.what()};
    } catch (const std::exception &e) {
      return {PlanStatus::Infeasible, {}, e.what()};
    }
  }

private:
  JointLimits limits_;
};
} // namespace motion
