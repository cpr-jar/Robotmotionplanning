#include "motion/kinematics/serial_chain_kinematics.hpp"
#include <cassert>
#ifndef MOTION_WITH_SERIAL
#error "Serial component must export its capability definition"
#endif
int main() {
  motion::SerialChainKinematics model(
      {{"x", "base", "tip", motion::SerialJointType::Prismatic, {}, {1, 0, 0}, -2, 2, 1}});
  const auto pose = model.fk({.25});
  assert(std::abs(pose.position[0] - .25) < 1e-12);
  const auto jacobian = model.jacobian({.25});
  assert(jacobian.size() == 6 && jacobian[0][0] == 1 && jacobian[1][0] == 0);
  const auto ik = model.ik(pose, {.2});
  assert(ik && std::abs((*ik)[0] - .25) < 1e-7);
}
