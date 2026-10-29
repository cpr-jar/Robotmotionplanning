#pragma once
#include "motion/robot_program.hpp"
namespace motion_test {
inline motion::ImportedSerialRobot xyz() {
  using namespace motion;
  const auto t = SerialJointType::Prismatic;
  auto model = std::make_shared<SerialChainKinematics>(
      std::vector<SerialJoint>{{"x", "base", "x", t, {}, {1, 0, 0}, -2, 2, 1},
                               {"y", "x", "y", t, {}, {0, 1, 0}, -2, 2, 1},
                               {"z", "y", "tool", t, {}, {0, 0, 1}, -2, 2, 1}});
  return {"xyz",
          "base",
          "tool",
          model,
          {Vector(3, -2), Vector(3, 2), Vector(3, 1), Vector(3, 2), Vector(3, 10)},
          {.2, .5, 2, .5, 1, 5}};
}
inline const std::string prefix =
    "schema_version: 1\nunits: SI\nrobot_profile: profile.yaml\nstart: {joints: [0.1, 0, 0]}\n";
} // namespace motion_test
