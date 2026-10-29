#pragma once
#include "motion/core/clock_time.hpp"
namespace motion {
struct ExecutionTimingPolicy {
  double max_early_s{}, max_late_s{}, max_clock_uncertainty_s{}, dispatch_latency_bound_s{};
};
enum class ExecutionStatus { Running, Finished, Inactive, Error };
enum class ExecutionFault {
  None,
  InvalidClock,
  ClockRegression,
  ExcessUncertainty,
  DeadlineMissed,
  Underflow,
  InvalidCommand
};
} // namespace motion
