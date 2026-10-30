#include "motion/kinematics/inverse_differential.hpp"
#include "motion/kinematics/reference_kinematics.hpp"
#include <cassert>
int main() {
  motion::ScaraKinematics model({.4, .3}, {{-3, -3, -1, -3}, {3, 3, 1, 3}});
  motion::Twist velocity;
  velocity.data[2] = .01;
  const auto inverse = motion::inverse_differential(model, {.3, .5, .1, .2}, velocity, {});
  assert(inverse && std::abs(inverse.boundary->dq[2] - .01) < 1e-12);
}
