#include "motion/calibration/rigid_calibration.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
namespace motion {
RigidCalibrationResult fit_rigid_calibration(const std::vector<PointCorrespondence> &points,
                                             double tolerance) {
  RigidCalibrationResult out;
  if (points.size() < 3 || points.size() > 100000 || !std::isfinite(tolerance) || tolerance <= 0) {
    out.diagnostic = "Invalid calibration input/tolerance";
    return out;
  }
  double weight = 0;
  Eigen::Vector3d source = Eigen::Vector3d::Zero(), target = source;
  for (auto p : points) {
    if (!std::isfinite(p.weight) || p.weight <= 0) {
      out.diagnostic = "Invalid correspondence weight";
      return out;
    }
    for (int i = 0; i < 3; ++i)
      if (!std::isfinite(p.source[i]) || !std::isfinite(p.target[i])) {
        out.diagnostic = "Nonfinite correspondence";
        return out;
      }
    weight += p.weight;
    source += p.weight * Eigen::Vector3d(p.source[0], p.source[1], p.source[2]);
    target += p.weight * Eigen::Vector3d(p.target[0], p.target[1], p.target[2]);
  }
  if (!std::isfinite(weight) || !source.allFinite() || !target.allFinite()) {
    out.diagnostic = "Calibration overflow";
    return out;
  }
  source /= weight;
  target /= weight;
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (auto p : points) {
    const Eigen::Vector3d a(p.source[0], p.source[1], p.source[2]), b(p.target[0], p.target[1], p.target[2]);
    covariance += p.weight * (a - source) * (b - target).transpose();
  }
  if (!covariance.allFinite()) {
    out.diagnostic = "Calibration covariance overflow";
    return out;
  }
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto singular = svd.singularValues();
  out.rank_margin = singular[0] > 0 ? singular[1] / singular[0] : 0;
  if (out.rank_margin < 1e-8) {
    out.status = CalibrationStatus::Degenerate;
    out.diagnostic = "Collinear/degenerate calibration points";
    return out;
  }
  Eigen::Matrix3d sign = Eigen::Matrix3d::Identity();
  if ((svd.matrixV() * svd.matrixU().transpose()).determinant() < 0)
    sign(2, 2) = -1;
  const Eigen::Matrix3d rotation = svd.matrixV() * sign * svd.matrixU().transpose();
  const Eigen::Vector3d translation = target - rotation * source;
  long double squared = 0;
  for (auto p : points) {
    const double error = (rotation * Eigen::Vector3d(p.source[0], p.source[1], p.source[2]) + translation -
                          Eigen::Vector3d(p.target[0], p.target[1], p.target[2]))
                             .norm();
    out.max_error_m = std::max(out.max_error_m, error);
    squared += static_cast<long double>(p.weight) * error * error;
  }
  out.rms_m = double(std::sqrt(squared / weight));
  if (!std::isfinite(out.rms_m) || out.max_error_m > tolerance) {
    out.status = CalibrationStatus::ResidualRejected;
    out.diagnostic = "Calibration residual exceeds supplied tolerance";
    return out;
  }
  const Eigen::Quaterniond q(rotation);
  out.target_from_source =
      Pose{{translation[0], translation[1], translation[2]}, {q.w(), q.x(), q.y(), q.z()}};
  out.status = CalibrationStatus::Ready;
  out.diagnostic = "Weighted proper rigid transform fitted; fit residual is not physical uncertainty";
  return out;
}
} // namespace motion
