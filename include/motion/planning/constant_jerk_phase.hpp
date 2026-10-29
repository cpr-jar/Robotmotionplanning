#pragma once
#include "motion/planning/boundary_quintic.hpp"
namespace motion::phase_detail {
struct Phase {
  long double begin{}, h{}, p{}, v{}, a{}, j{};
  std::array<long double, 3> at(long double t) const {
    return {p + v * t + a * t * t / 2 + j * t * t * t / 6, v + a * t + j * t * t / 2, a + j * t};
  }
};
struct Bounds {
  double min_position{INFINITY}, max_position{-INFINITY}, max_velocity{}, max_acceleration{}, max_jerk{};
};
inline bool check(const Phase &phase, double lo, double hi, double vcap, double acap, double jcap,
                  Bounds &b) {
  for (long double x : {phase.begin, phase.h, phase.p, phase.v, phase.a, phase.j})
    if (!std::isfinite(x))
      return false;
  if (phase.h < 0)
    return false;
  const long double tolerance = 128 * std::numeric_limits<double>::epsilon();
  bool good = true;
  const auto inspect = [&](long double t) {
    if (t < 0 || t > phase.h)
      return;
    const auto x = phase.at(t);
    b.min_position = std::min(b.min_position, double(x[0]));
    b.max_position = std::max(b.max_position, double(x[0]));
    b.max_velocity = std::max(b.max_velocity, double(std::abs(x[1])));
    b.max_acceleration = std::max(b.max_acceleration, double(std::abs(x[2])));
    b.max_jerk = std::max(b.max_jerk, double(std::abs(phase.j)));
    if (x[0] < lo || x[0] > hi || std::abs(x[1]) > vcap * (1 + tolerance) ||
        std::abs(x[2]) > acap * (1 + tolerance) || std::abs(phase.j) > jcap * (1 + tolerance))
      good = false;
  };
  inspect(0);
  inspect(phase.h);
  if (phase.j != 0)
    inspect(-phase.a / phase.j);
  if (phase.j == 0) {
    if (phase.a != 0)
      inspect(-phase.v / phase.a);
  } else {
    const auto disc = phase.a * phase.a - 2 * phase.j * phase.v;
    if (disc >= 0) {
      const auto root = std::sqrt(disc);
      inspect((-phase.a - root) / phase.j);
      inspect((-phase.a + root) / phase.j);
    }
  }
  return good;
}
} // namespace motion::phase_detail
