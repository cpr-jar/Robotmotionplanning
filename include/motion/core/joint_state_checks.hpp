#pragma once
#include "motion/core/motion_types.hpp"
namespace motion {
inline bool is_rest(const JointState &s, double tolerance = 1e-10) {
  for (double x : s.dq)
    if (!std::isfinite(x) || std::abs(x) > tolerance)
      return false;
  for (double x : s.ddq)
    if (!std::isfinite(x) || std::abs(x) > tolerance)
      return false;
  return true;
}
inline bool is_rest(const JointTarget &s, double tolerance = 1e-10) {
  for (double x : s.dq)
    if (!std::isfinite(x) || std::abs(x) > tolerance)
      return false;
  for (double x : s.ddq)
    if (!std::isfinite(x) || std::abs(x) > tolerance)
      return false;
  return true;
}

} // namespace motion
