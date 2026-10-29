#pragma once
#include "motion/kinematics/serial_chain_kinematics.hpp"
#include <map>
#include <memory>
#include <string_view>
namespace motion {
class ModelImportError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};
struct JointProfile {
  double max_acceleration{}, max_jerk{};
  std::optional<double> lower, upper, max_velocity;
};
struct SerialRobotProfile {
  std::string robot_name, base_link, tip_link;
  Pose base_pose, tool_offset;
  std::map<std::string, JointProfile> joints;
  CartesianLimits tcp_limits;
  SerialIKOptions solver;
};
struct ImportedSerialRobot {
  std::string name, base_link, tip_link;
  std::shared_ptr<const SerialChainKinematics> kinematics;
  JointLimits joint_limits;
  CartesianLimits tcp_limits;
};
SerialRobotProfile parse_serial_profile_yaml(std::string_view);
ImportedSerialRobot import_serial_urdf(std::string_view, const SerialRobotProfile &);
ImportedSerialRobot load_serial_robot_files(const std::string &, const std::string &);
} // namespace motion
