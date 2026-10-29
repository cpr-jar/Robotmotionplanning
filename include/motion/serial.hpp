#pragma once
#include "kinematics.hpp"
namespace motion {
enum class SerialJointType { Fixed, Revolute, Continuous, Prismatic };
struct SerialJoint {
  std::string name, parent_link, child_link;
  SerialJointType type{SerialJointType::Fixed};
  Pose origin;
  std::array<double, 3> axis{1, 0, 0};
  double lower{}, upper{}, max_velocity{};
};
struct SerialIKOptions {
  std::size_t max_iterations{300};
  double position_tolerance_m{1e-7}, orientation_tolerance_rad{1e-7}, orientation_length_m{.25},
      prismatic_scale_m{.1}, damping{1e-3}, max_normalized_step{.25}, singularity_threshold{1e-6};
};
enum class SerialIKStatus {
  Success,
  InvalidInput,
  OutsideReachBound,
  SingularSolution,
  StalledAtLimits,
  NoConvergence,
  NumericalFailure
};
struct SerialIKResult {
  SerialIKStatus status{SerialIKStatus::InvalidInput};
  Vector solution;
  std::size_t iterations{};
  double position_error_m{}, orientation_error_rad{}, rank_margin{};
  std::string diagnostic;
  explicit operator bool() const noexcept { return status == SerialIKStatus::Success; }
};
class SerialChainKinematics final : public IDifferentialKinematics {
public:
  SerialChainKinematics(std::vector<SerialJoint>, Pose base = {}, Pose tool = {}, SerialIKOptions = {});
  std::size_t dof() const override { return names_.size(); }
  const KinematicJointLimits &position_limits() const noexcept override { return limits_; }
  const std::vector<std::string> &joint_names() const noexcept { return names_; }
  const std::vector<SerialJoint> &joints() const noexcept { return joints_; }
  const Pose &base_pose() const noexcept { return base_; }
  const Pose &tool_offset() const noexcept { return tool_; }
  const SerialIKOptions &solver_options() const noexcept { return options_; }
  Pose fk(const Vector &) const override;
  std::vector<Vector> jacobian(const Vector &) const override;
  SingularityInfo singularity(const Vector &) const override;
  KinematicDerivatives differentiate(const Vector &, const Vector &, const Vector &,
                                     const Vector &) const override;
  SerialIKResult ik_detailed(const Pose &, const Vector &) const;
  std::optional<Vector> ik(const Pose &p, const Vector &seed) const override {
    auto r = ik_detailed(p, seed);
    return r ? std::optional<Vector>(r.solution) : std::nullopt;
  }

private:
  void require_q(const Vector &) const;
  std::vector<SerialJoint> joints_;
  Pose base_, tool_;
  SerialIKOptions options_;
  KinematicJointLimits limits_;
  std::vector<std::string> names_;
};
} // namespace motion
