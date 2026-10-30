#pragma once
#include "motion/math/rigid_transform.hpp"
#include <limits>
#include <numbers>
namespace motion {
struct KinematicJointLimits {
  Vector lower, upper;
};
struct SingularityInfo {
  double rank_margin{1}, transmission_margin{1};
  bool singular{};
};
class IDifferentialKinematics : public IKinematics {
public:
  virtual const KinematicJointLimits &position_limits() const noexcept = 0;
  virtual SingularityInfo singularity(const Vector &) const = 0;
  virtual KinematicDerivatives differentiate(const Vector &, const Vector &, const Vector &,
                                             const Vector &) const = 0;
};
} // namespace motion
