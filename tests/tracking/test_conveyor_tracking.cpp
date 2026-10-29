#include "motion/tracking/conveyor_target_predictor.hpp"
#include <cassert>
#include <iostream>
using namespace motion;
int main() {
  const auto time = [](double t) { return ClockTime{t, 7, .000001}; };
  TrackingFrameSnapshot frames(
      "world",
      {{"camera", "world", {{.1, 0, 0}, {1, 0, 0, 0}}, .00001, .00001},
       {"belt",
        "world",
        {{0, 0, 0}, {std::cos(kinematic_detail::pi / 4), 0, 0, std::sin(kinematic_detail::pi / 4)}},
        0,
        .00001}},
      1, {0, 7, 0}, {100, 7, 0});
  ConveyorTargetPredictor predictor({"belt"});
  ObjectObservation object{"part", "camera", {{.2, 0, 0}, {1, 0, 0, 0}}, time(10), time(10.01),
                           .00001, .00001};
  EncoderObservation encoder{20, .1, .02, time(10), time(10.01), .00001, .00001};
  const auto predicted = predictor.predict(object, encoder, frames, time(10.02), time(10.3));
  assert(predicted);
  assert(std::abs(predicted.target->pose.position[0] - .3) < 1e-12);
  assert(std::abs(predicted.target->pose.position[1] - (.1 * .3 + .5 * .02 * .3 * .3)) < 1e-12);
  assert(std::abs(predicted.target->velocity[1] - .106) < 1e-12);
  encoder.position_m += 100;
  const auto offset = predictor.predict(object, encoder, frames, time(10.02), time(10.3));
  assert(offset && offset.target->pose.position == predicted.target->pose.position);
  assert(!predictor.predict(object, encoder, frames, time(11), time(11.3)));
  auto future = object;
  future.received_at = time(9);
  assert(!predictor.predict(future, encoder, frames, time(10.02), time(10.3)));
  auto wrong = encoder;
  wrong.measured_at.clock = 8;
  assert(!predictor.predict(object, wrong, frames, time(10.02), time(10.3)));
  assert(!predictor.predict(object, encoder, frames, time(10.02), time(12)));
  bool cycle = false;
  try {
    TrackingFrameSnapshot bad("world", {{"a", "b", {}, 0, 0}, {"b", "a", {}, 0, 0}}, 1, {0, 7, 0},
                              {100, 7, 0});
  } catch (const std::exception &) {
    cycle = true;
  }
  assert(cycle);
  std::cout << "Calibrated frame rotation/composition, positive clock uncertainty, offset cancellation and "
               "stale/causal/domain/horizon rejection passed\n";
}
