#pragma once
#include <array>
namespace motion {
using PathVector3 = std::array<double, 3>;
struct Pose {
  std::array<double, 3> position{};
  std::array<double, 4> orientation{1, 0, 0, 0};
};
struct Twist {
  std::array<double, 6> data{};
};
struct KinematicDerivatives {
  Twist velocity, acceleration, jerk;
};
} // namespace motion
