#pragma once
#include "motion/kinematics/differential_kinematics.hpp"
namespace motion {
struct PointCorrespondence {
  std::array<double, 3> source{}, target{};
  double weight{1};
};
enum class CalibrationStatus { Ready, InvalidInput, Degenerate, ResidualRejected };
struct RigidCalibrationResult {
  CalibrationStatus status{CalibrationStatus::InvalidInput};
  std::optional<Pose> target_from_source;
  double rms_m{}, max_error_m{}, rank_margin{};
  std::string diagnostic;
  explicit operator bool() const { return status == CalibrationStatus::Ready && bool(target_from_source); }
};
RigidCalibrationResult fit_rigid_calibration(const std::vector<PointCorrespondence> &,
                                             double maximum_error_m);
} // namespace motion
