#include "motion/tracking/constant_velocity_predictor.hpp"
#include "motion/tracking/conveyor_target_predictor.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using namespace motion;
namespace {
void legacy_prediction() {
  ConveyorPredictor predictor(1);
  const TrackedTarget target{{{.2, .3, .4}, {1, 0, 0, 0}}, 10};
  const ConveyorState belt{{1, 2, 3}, {2, 0, 0}, .1, .2, .4, 10};
  const auto pose = predictor.predict(target, belt, 10.5);
  assert(pose && std::abs(pose->position[0] - 1.45) < 1e-12);
  assert(pose->position[1] == 2.3 && pose->orientation == target.pose_in_conveyor.orientation);
  for (double bad : {NAN, INFINITY, -INFINITY}) {
    assert(!predictor.predict(target, belt, bad));
    bool rejected = false;
    try {
      ConveyorPredictor invalid(bad);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    assert(rejected);
    for (auto member : {&ConveyorState::position_m, &ConveyorState::velocity_m_s,
                        &ConveyorState::acceleration_m_s2, &ConveyorState::observed_at_s}) {
      auto wrong = belt;
      wrong.*member = bad;
      assert(!predictor.predict(target, wrong, 10.5));
    }
    auto wrong_target = target;
    wrong_target.observed_at_s = bad;
    assert(!predictor.predict(wrong_target, belt, 10.5));
    for (std::size_t axis = 0; axis < 3; ++axis) {
      auto wrong = belt;
      wrong.origin_world[axis] = bad;
      assert(!predictor.predict(target, wrong, 10.5));
      wrong = belt;
      wrong.direction_world[axis] = bad;
      assert(!predictor.predict(target, wrong, 10.5));
      wrong_target = target;
      wrong_target.pose_in_conveyor.position[axis] = bad;
      assert(!predictor.predict(wrong_target, belt, 10.5));
    }
    for (std::size_t axis = 0; axis < 4; ++axis) {
      wrong_target = target;
      wrong_target.pose_in_conveyor.orientation[axis] = bad;
      assert(!predictor.predict(wrong_target, belt, 10.5));
    }
  }
  auto overflow = belt;
  overflow.velocity_m_s = overflow.acceleration_m_s2 = std::numeric_limits<double>::max();
  assert(!predictor.predict(target, overflow, 11));
  auto zero_axis = belt;
  zero_axis.direction_world.fill(0);
  assert(!predictor.predict(target, zero_axis, 10.5));
  assert(!predictor.predict(target, belt, 9));
  assert(!predictor.predict(target, belt, 12));
}
} // namespace
int main() {
  legacy_prediction();
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
