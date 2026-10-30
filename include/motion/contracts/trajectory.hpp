#pragma once
#include "motion/core/motion_types.hpp"
#include <memory>
namespace motion {
// A trajectory is immutable once published. sample() may allocate and throw:
// sample in a preparation/producer thread, then dispatch fixed-size commands.
// Immutability alone does not make a trajectory suitable for a realtime callback.
class ITrajectory {
public:
  virtual ~ITrajectory() = default;
  virtual double duration() const noexcept = 0;
  virtual JointState sample(double time_s) const = 0;
};
using TrajectoryPtr = std::shared_ptr<const ITrajectory>;

enum class PlanStatus { Ready, InvalidInput, Unsupported, Infeasible };
struct PlanResult {
  PlanStatus status{PlanStatus::InvalidInput};
  TrajectoryPtr trajectory;
  std::string diagnostic;
  explicit operator bool() const noexcept { return status == PlanStatus::Ready && trajectory != nullptr; }
};
class IJointTrajectoryPlanner {
public:
  virtual ~IJointTrajectoryPlanner() = default;
  virtual PlanResult plan(const JointState &initial, const JointTarget &target) const = 0;
};
struct CartesianRequest {
  Pose start, goal;
  CartesianLimits limits;
};
class ICartesianPathPlanner {
public:
  virtual ~ICartesianPathPlanner() = default;
  virtual PlanResult plan(const CartesianRequest &request, const JointState &initial) const = 0;
};

} // namespace motion
