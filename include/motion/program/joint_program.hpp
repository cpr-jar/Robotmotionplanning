#pragma once
#include "motion/planning/joint_trajectory.hpp"
namespace motion {
class IJointProgram : public ITrajectory {
public:
  virtual std::size_t segment_count() const noexcept = 0;
  virtual double segment_end(std::size_t) const = 0;
  virtual double segment_duration(std::size_t) const = 0;
  virtual JointState sample_segment(std::size_t, double) const = 0;
  virtual const JointLimits &joint_limits() const noexcept = 0;
  virtual const JointState &initial_state() const noexcept = 0;
};
} // namespace motion
