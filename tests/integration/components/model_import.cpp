#include "motion/io/robot_model_import.hpp"
#include <cassert>
int main() {
  const std::string directory = std::string(MOTION_SOURCE_DIR) + "/models/ur5e/";
  const auto robot = motion::load_serial_robot_files(directory + "robot.urdf", directory + "profile.yaml");
  assert(robot.kinematics->dof() == 6);
  const auto pose = robot.kinematics->fk(motion::Vector(6));
  assert(std::isfinite(pose.position[0]) && std::isfinite(pose.position[2]));
}
