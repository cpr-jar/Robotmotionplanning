#pragma once
#include "motion/core/motion_types.hpp"
namespace motion {
// Predicts a moving Cartesian target; does not by itself plan a feasible approach.
struct TrackedTarget {
  Pose pose_in_conveyor;
  double observed_at_s{};
};
struct ConveyorState {
  std::array<double, 3> origin_world{};
  std::array<double, 3> direction_world{1, 0, 0};
  double position_m{};
  double velocity_m_s{};
  double acceleration_m_s2{};
  double observed_at_s{};
};
class ConveyorPredictor {
public:
  explicit ConveyorPredictor(double max_age_s) : max_age_s_(max_age_s) {
    if (max_age_s_ <= 0)
      throw std::invalid_argument("max_age_s must be positive");
  }
  std::optional<Pose> predict(const TrackedTarget &target, const ConveyorState &belt,
                              double future_time_s) const {
    if (future_time_s < belt.observed_at_s || future_time_s - belt.observed_at_s > max_age_s_ ||
        future_time_s < target.observed_at_s || future_time_s - target.observed_at_s > max_age_s_)
      return std::nullopt;
    auto result = target.pose_in_conveyor;
    const double dt = future_time_s - belt.observed_at_s;
    const double displacement =
        belt.position_m + belt.velocity_m_s * dt + 0.5 * belt.acceleration_m_s2 * dt * dt;
    double norm = 0;
    for (double v : belt.direction_world)
      norm += v * v;
    if (norm < 1e-12)
      return std::nullopt;
    norm = std::sqrt(norm);
    for (std::size_t i = 0; i < 3; ++i)
      result.position[i] += belt.origin_world[i] + displacement * belt.direction_world[i] / norm;
    // Assumes belt frame has no rotation and constant acceleration in prediction horizon.
    return result;
  }

private:
  double max_age_s_;
};
} // namespace motion
