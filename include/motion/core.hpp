#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
namespace motion {
using Vector = std::vector<double>;
struct JointState {
  Vector q, dq, ddq;
  double timestamp_s{};
};
struct JointLimits {
  Vector min_position, max_position, max_velocity, max_acceleration, max_jerk;
};
struct Pose {
  std::array<double, 3> position{};
  std::array<double, 4> orientation{1, 0, 0, 0};
};
struct Twist {
  std::array<double, 6> data{};
};
struct CartesianLimits {
  double max_linear_velocity{}, max_linear_acceleration{}, max_linear_jerk{};
  double max_angular_velocity{}, max_angular_acceleration{}, max_angular_jerk{};
};
struct JointTarget {
  Vector q, dq, ddq;
};
struct MotionSample {
  JointState joints;
  std::optional<Pose> tcp;
};
enum class StepStatus { Running, Finished, InvalidInput, Error };
inline void check_limits(const JointLimits &l, std::size_t n) {
  const auto same = [n](const Vector &v) { return v.size() == n; };
  if (!n || !same(l.min_position) || !same(l.max_position) || !same(l.max_velocity) ||
      !same(l.max_acceleration) || !same(l.max_jerk))
    throw std::invalid_argument("Invalid joint limit dimension");
  for (std::size_t i = 0; i < n; ++i)
    if (!(std::isfinite(l.min_position[i]) && std::isfinite(l.max_position[i]) &&
          std::isfinite(l.max_velocity[i]) && std::isfinite(l.max_acceleration[i]) &&
          std::isfinite(l.max_jerk[i]) && l.min_position[i] < l.max_position[i] && l.max_velocity[i] > 0 &&
          l.max_acceleration[i] > 0 && l.max_jerk[i] > 0))
      throw std::invalid_argument("Invalid joint limit value");
}
inline void check_state(const JointState &s, std::size_t n) {
  if (s.q.size() != n || s.dq.size() != n || s.ddq.size() != n)
    throw std::invalid_argument("Invalid joint state dimension");
}
// Limits on commanded motion; independent hardware safety supervision still required.
inline bool within_joint_limits(const JointState &s, const JointLimits &l, double epsilon = 1e-8) {
  check_limits(l, s.q.size());
  check_state(s, s.q.size());
  for (std::size_t i = 0; i < s.q.size(); ++i)
    if (!std::isfinite(s.q[i]) || !std::isfinite(s.dq[i]) || !std::isfinite(s.ddq[i]) ||
        s.q[i] < l.min_position[i] - epsilon || s.q[i] > l.max_position[i] + epsilon ||
        std::abs(s.dq[i]) > l.max_velocity[i] + epsilon ||
        std::abs(s.ddq[i]) > l.max_acceleration[i] + epsilon)
      return false;
  return true;
}
// An IK solution must remain on a continuous branch; q_seed is mandatory.
class IKinematics {
public:
  virtual ~IKinematics() = default;
  virtual std::size_t dof() const = 0;
  virtual Pose fk(const Vector &q) const = 0;
  virtual std::optional<Vector> ik(const Pose &target, const Vector &q_seed) const = 0;
  virtual std::vector<Vector> jacobian(const Vector &q) const = 0; // 6 x DoF, linear then angular
};
// XYZ cartesian gantry with fixed TCP orientation; units: meters and radians.
class CartesianGantry final : public IKinematics {
public:
  std::size_t dof() const override { return 3; }
  Pose fk(const Vector &q) const override {
    if (!finite_joints(q))
      throw std::invalid_argument("Gantry requires 3 finite joints");
    return {{q[0], q[1], q[2]}, {1, 0, 0, 0}};
  }
  std::optional<Vector> ik(const Pose &p, const Vector &seed) const override {
    if (!finite_joints(seed) ||
        !std::all_of(p.position.begin(), p.position.end(), [](double x) { return std::isfinite(x); }) ||
        !std::all_of(p.orientation.begin(), p.orientation.end(), [](double x) { return std::isfinite(x); }))
      return std::nullopt;
    if (std::abs(std::abs(p.orientation[0]) - 1) > 1e-9 || std::abs(p.orientation[1]) > 1e-9 ||
        std::abs(p.orientation[2]) > 1e-9 || std::abs(p.orientation[3]) > 1e-9)
      return std::nullopt;
    return Vector{p.position[0], p.position[1], p.position[2]};
  }
  std::vector<Vector> jacobian(const Vector &q) const override {
    if (!finite_joints(q))
      throw std::invalid_argument("Gantry requires 3 finite joints");
    return {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  }

private:
  static bool finite_joints(const Vector &q) {
    return q.size() == 3 && std::all_of(q.begin(), q.end(), [](double x) { return std::isfinite(x); });
  }
};
// Generic adapter for a serial or Delta kinematic solver supplied by the robot plugin.
// A real plugin must implement FK, branch-continuous IK, and Jacobian.
struct RobotModel {
  std::string name;
  JointLimits limits;
  const IKinematics *kinematics{}; // lifetime managed by application
  void validate() const {
    if (!kinematics)
      throw std::invalid_argument("Kinematics missing");
    check_limits(limits, kinematics->dof());
  }
};
} // namespace motion
