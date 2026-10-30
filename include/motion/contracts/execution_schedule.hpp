#pragma once
#include "motion/contracts/trajectory.hpp"
#include <cstdint>
namespace motion {
/// Immutable timing/limit contract. sample()/sample_tick() belong outside the cyclic thread.
class IExecutionSchedule {
public:
  virtual ~IExecutionSchedule() = default;
  virtual double duration() const noexcept = 0;
  virtual double cycle_s() const noexcept = 0;
  virtual std::uint64_t total_ticks() const noexcept = 0;
  virtual double tick_time(std::uint64_t) const noexcept = 0;
  virtual const std::vector<std::uint64_t> &boundary_ticks() const noexcept = 0;
  virtual const JointState &initial_state() const noexcept = 0;
  virtual const JointLimits &joint_limits() const noexcept = 0;
  virtual JointState sample(double) const = 0;
  virtual JointState sample_tick(std::uint64_t) const = 0;
};
namespace schedule_detail {
constexpr std::uint64_t max_ticks = std::uint64_t(1) << 52;
inline double tick_time(std::uint64_t tick, double cycle) {
  return double(static_cast<long double>(tick) * cycle);
}
inline bool valid_clock(double epoch, double cycle, std::uint64_t ticks) {
  const double end = epoch + tick_time(ticks, cycle), mag = std::max(std::abs(epoch), std::abs(end));
  return std::isfinite(epoch) && std::isfinite(end) && std::isfinite(cycle) && cycle > 0 && ticks > 0 &&
         ticks <= max_ticks && cycle > 4 * (std::nextafter(mag, INFINITY) - mag) && epoch + cycle > epoch;
}
} // namespace schedule_detail
} // namespace motion
