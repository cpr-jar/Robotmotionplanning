#pragma once
#include "motion/core/clock_time.hpp"
#ifdef __linux__
#include <cerrno>
#include <cmath>
#include <ctime>
#include <limits>
#include <optional>
#include <sched.h>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <vector>
namespace motion {
class LinuxMonotonicClock {
public:
  explicit LinuxMonotonicClock(ClockId clock = 7, double uncertainty = .000001)
      : clock_id_(clock), clock_uncertainty_s_(uncertainty) {
    if (!clock || !std::isfinite(uncertainty) || uncertainty < 0)
      throw std::invalid_argument("Invalid monotonic clock policy");
  }
  ClockTime now() const noexcept {
    timespec stamp{};
    if (::clock_gettime(CLOCK_MONOTONIC, &stamp) != 0)
      return {NAN, clock_id_, INFINITY};
    const double seconds = double(static_cast<long double>(stamp.tv_sec) + stamp.tv_nsec * 1e-9L);
    return {seconds, clock_id_,
            clock_uncertainty_s_ + 8 * std::numeric_limits<double>::epsilon() * std::abs(seconds)};
  }
  int wait_until(ClockTime deadline) const noexcept {
    if (deadline.clock != clock_id_ || !std::isfinite(deadline.seconds) || deadline.seconds < 0 ||
        !std::isfinite(deadline.uncertainty_s) || deadline.uncertainty_s < 0)
      return EINVAL;
    const long double seconds = std::floor(static_cast<long double>(deadline.seconds));
    if (seconds >= std::numeric_limits<time_t>::max())
      return EOVERFLOW;
    timespec stamp{time_t(seconds),
                   long(std::ceil((static_cast<long double>(deadline.seconds) - seconds) * 1e9L))};
    if (stamp.tv_nsec >= 1000000000L) {
      ++stamp.tv_sec;
      stamp.tv_nsec = 0;
    }
    return ::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &stamp, nullptr);
  }

private:
  ClockId clock_id_;
  double clock_uncertainty_s_;
};
struct RealtimeThreadOptions {
  std::optional<int> fifo_priority, cpu;
  bool lock_memory{};
};
struct RealtimeSetupResult {
  std::vector<std::string> errors;
  explicit operator bool() const { return errors.empty(); }
};
inline RealtimeSetupResult configure_realtime_thread(const RealtimeThreadOptions &options) {
  RealtimeSetupResult result;
  if (options.cpu) {
    const int cpu = *options.cpu;
    if (cpu < 0 || cpu >= CPU_SETSIZE)
      result.errors.push_back("Invalid CPU affinity index");
    else {
      cpu_set_t set;
      CPU_ZERO(&set);
      CPU_SET(cpu, &set);
      if (::sched_setaffinity(0, sizeof(set), &set) != 0)
        result.errors.push_back("sched_setaffinity errno=" + std::to_string(errno));
    }
  }
  if (options.fifo_priority) {
    const int value = *options.fifo_priority;
    if (value < ::sched_get_priority_min(SCHED_FIFO) || value > ::sched_get_priority_max(SCHED_FIFO))
      result.errors.push_back("Invalid FIFO priority");
    else {
      sched_param parameters{};
      parameters.sched_priority = value;
      if (::sched_setscheduler(0, SCHED_FIFO, &parameters) != 0)
        result.errors.push_back("sched_setscheduler errno=" + std::to_string(errno));
    }
  }
  if (options.lock_memory && ::mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    result.errors.push_back("mlockall errno=" + std::to_string(errno));
  return result;
}
struct CycleTimingStatistics {
  std::uint64_t samples{}, missed_periods{};
  double maximum_lateness_s{};
  long double sum_lateness_s{};
  void observe(double due, double actual, double cycle) {
    if (!std::isfinite(due) || !std::isfinite(actual) || !std::isfinite(cycle) || cycle <= 0)
      throw std::invalid_argument("Invalid timing sample");
    const double late = std::max(0., actual - due);
    const long double missed = std::floor(static_cast<long double>(late) / cycle);
    if (samples == UINT64_MAX || missed > UINT64_MAX - missed_periods)
      throw std::overflow_error("Timing statistics overflow");
    ++samples;
    missed_periods += std::uint64_t(missed);
    maximum_lateness_s = std::max(maximum_lateness_s, late);
    sum_lateness_s += late;
  }
  double average_lateness_s() const { return samples ? double(sum_lateness_s / samples) : 0; }
};
} // namespace motion
#endif
