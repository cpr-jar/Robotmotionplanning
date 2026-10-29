#pragma once
#include <cstdint>
namespace motion {
using ClockId = std::uint64_t;
/// Time in seconds in an explicit monotonic clock domain; uncertainty is an absolute bound.
struct ClockTime {
  double seconds{};
  ClockId clock{};
  double uncertainty_s{};
};
// Source-compatible name retained for existing tracking and execution applications.
using TrackingTime = ClockTime;
} // namespace motion
