#include "motion/linux_platform.hpp"
#include <charconv>
#include <iostream>
int main(int argc, char **argv) {
  using namespace motion;
  std::uint64_t samples = 100;
  if (argc == 2) {
    const std::string value = argv[1];
    const auto parsed = std::from_chars(value.data(), value.data() + value.size(), samples);
    if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() || !samples || samples > 100000)
      return 2;
  } else if (argc != 1)
    return 2;
  LinuxMonotonicClock clock;
  const auto start = clock.now();
  CycleTimingStatistics stats;
  for (std::uint64_t tick = 1; tick <= samples; ++tick) {
    const double due = start.seconds + schedule_detail::tick_time(tick, .01);
    const int result = clock.wait_until({due, 7, 0});
    if (result) {
      std::cerr << "Absolute wait returned errno=" << result << '\n';
      return 1;
    }
    const auto now = clock.now();
    stats.observe(due, now.seconds, .01);
  }
  std::cout << "10 ms host timer probe: samples=" << stats.samples
            << ", maximum lateness=" << stats.maximum_lateness_s
            << " s, missed whole periods=" << stats.missed_periods
            << ". Diagnostic only; no hardware or hard-realtime proof.\n";
}
