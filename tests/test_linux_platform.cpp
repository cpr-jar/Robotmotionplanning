#include "motion/linux_platform.hpp"
#include <cassert>
#include <iostream>
int main() {
  using namespace motion;
  LinuxMonotonicClock clock;
  auto first = clock.now(), last = clock.now();
  assert(std::isfinite(first.seconds) && last.seconds >= first.seconds && last.clock == 7 &&
         last.uncertainty_s > 0);
  assert(clock.wait_until({0, 7, 0}) == 0);
  assert(clock.wait_until({0, 8, 0}) == EINVAL);
  assert(configure_realtime_thread({}));
  assert(!configure_realtime_thread({-1, {}, false}));
  CycleTimingStatistics stats;
  stats.observe(10, 10.001, .01);
  stats.observe(11, 11.021, .01);
  assert(stats.samples == 2 && stats.missed_periods == 2 && stats.maximum_lateness_s > .02);
  assert(std::abs(stats.average_lateness_s() - .011) < 1e-12);
  std::cout
      << "Monotonic clock, absolute past deadline, explicit setup errors and timing statistics passed\n";
}
