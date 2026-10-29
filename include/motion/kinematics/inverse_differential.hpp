#pragma once
#include "motion/kinematics/differential_kinematics.hpp"
namespace motion {
struct InverseDifferentialResult {
  std::optional<JointTarget> boundary;
  double velocity_residual{}, acceleration_residual{}, rank_margin{};
  std::string diagnostic;
  explicit operator bool() const { return bool(boundary); }
};
InverseDifferentialResult inverse_differential(const IDifferentialKinematics &, const Vector &, const Twist &,
                                               const Twist &);
} // namespace motion
