#include "motion/calibration/rigid_calibration.hpp"
#include <cassert>
int main() {
  const auto fit = motion::fit_rigid_calibration({{{0, 0, 0}, {1, 2, 3}, 1},
                                                  {{1, 0, 0}, {2, 2, 3}, 1},
                                                  {{0, 1, 0}, {1, 3, 3}, 1},
                                                  {{0, 0, 1}, {1, 2, 4}, 1}},
                                                 1e-8);
  assert(fit && fit.rms_m < 1e-12 && std::abs(fit.target_from_source->position[0] - 1) < 1e-12);
}
