#include "motion/platform/linux_realtime.hpp"
#include <charconv>
#include <iostream>
int main(int argc, char **argv) {
  using namespace motion;
  constexpr double cycle_period_s = .01;
  std::uint64_t sample_count = 100;
  if (argc == 2) {
    const std::string value = argv[1];
    const auto parsed = std::from_chars(value.data(), value.data() + value.size(), sample_count);
    if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() || !sample_count ||
        sample_count > 100000)
      return 2;
  } else if (argc != 1)
    return 2;
  LinuxMonotonicClock clock;
  const auto start_time = clock.now();
  CycleTimingStatistics timing_statistics;
  for (std::uint64_t tick = 1; tick <= sample_count; ++tick) {
    const double deadline_s = start_time.seconds + double(static_cast<long double>(tick) * cycle_period_s);
    const int wait_result = clock.wait_until({deadline_s, start_time.clock, 0});
    if (wait_result) {
      std::cerr << "Absolute wait returned errno=" << wait_result << '\n';
      return 1;
    }
    const auto observed_time = clock.now();
    timing_statistics.observe(deadline_s, observed_time.seconds, cycle_period_s);
  }
  std::cout << "10 ms host timer probe: samples=" << timing_statistics.samples
            << ", maximum lateness=" << timing_statistics.maximum_lateness_s
            << " s, missed whole periods=" << timing_statistics.missed_periods
            << ". Diagnostic only; no hardware or hard-realtime proof.\n";
}
