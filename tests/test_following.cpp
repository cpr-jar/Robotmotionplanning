#include "motion/conveyor_following.hpp"
#ifdef MOTION_WITH_SERIAL
#include "motion/model_import.hpp"
#endif
#include <cassert>
#include <iostream>
using namespace motion;
namespace {
TrackingTime stamp(double t) {
  return {t, 7, 0};
}
#ifdef MOTION_WITH_SERIAL
double displacement(double t) {
  return .002 * t + .0001 * (1 - std::cos(t));
}
double velocity(double t) {
  return .002 + .0001 * std::sin(t);
}
double acceleration(double t) {
  return .0001 * std::cos(t);
}
template <std::size_t N> JointState state(const FixedJointCommand<N> &c) {
  return {Vector(c.q.begin(), c.q.end()), Vector(c.dq.begin(), c.dq.end()),
          Vector(c.ddq.begin(), c.ddq.end()), c.timestamp_s};
}
void serial_follow(const char *name) {
  const auto directory = std::string(MOTION_SOURCE_DIR) + "/models/" + name;
  const auto robot = load_serial_robot_files(directory + "/robot.urdf", directory + "/profile.yaml");
  const Vector q{.2, -.6, .7, .3, .8, -.2};
  const auto origin = robot.kinematics->fk(q);
  TrackingFrameSnapshot frames("world", {}, 1, stamp(0), stamp(100));
  ConveyorObservationHistory history("part", 1, {"world"});
  const auto observe = [&](double t) {
    auto pose = origin;
    pose.position[0] += displacement(t);
    return history.ingest({"part", "world", pose, stamp(10 + t), stamp(10 + t), .00001, 0},
                          {displacement(t), velocity(t), acceleration(t), stamp(10 + t), stamp(10 + t), 0, 0},
                          frames, stamp(10 + t));
  };
  assert(observe(0));
  Twist v{}, a{};
  v.data[0] = velocity(0);
  a.data[0] = acceleration(0);
  auto boundary = inverse_differential(*robot.kinematics, q, v, a);
  assert(boundary);
  auto forward = robot.kinematics->differentiate(q, boundary.boundary->dq, boundary.boundary->ddq, Vector(6));
  for (int i = 0; i < 6; ++i) {
    assert(std::abs(forward.velocity.data[i] - v.data[i]) < 1e-10);
    assert(std::abs(forward.acceleration.data[i] - a.data[i]) < 1e-10);
  }
  const JointState initial{q, boundary.boundary->dq, boundary.boundary->ddq, 10};
  KinematicConveyorFollower<6> follower(robot.kinematics, robot.joint_limits, {2, 10, 100, 2, 10, 100});
  auto active = follower.plan(initial, 10, .01, {1, 0, 0, 7}, 40, history, frames, stamp(10));
  if (!active) {
    std::cerr << name << ": " << active.diagnostic << '\n';
    std::abort();
  }
  KinematicConveyorFollower<6> too_slow(robot.kinematics, robot.joint_limits, {.0001, 10, 100, 2, 10, 100});
  assert(!too_slow.plan(initial, 10, .01, {1, 0, 0, 7}, 40, history, frames, stamp(10)));
  KinematicConveyorFollower<6> bad_stop(robot.kinematics, robot.joint_limits, {2, 10, 100, .00001, 10, 100});
  const auto rejected = bad_stop.plan(initial, 10, .01, {1, 0, 0, 7}, 40, history, frames, stamp(10));
  if (rejected || rejected.diagnostic.rfind("Following fallback:", 0) != 0) {
    std::cerr << name << " fallback gate: " << rejected.diagnostic << '\n';
    std::abort();
  }
  std::vector<std::shared_ptr<const PreparedLivePlan<6>>> retained{active.plan};
  LivePlanMailbox<6> box;
  LiveMotionExecutor<6> execution(*active.plan, box, {0, .001, 0, 0});
  LiveJointCommand<6> output;
  std::uint64_t tick = 0, generation = 1, updates = 0;
  double error = 0;
  for (; tick < 150; ++tick) {
    LivePlanResponse response;
    if (box.try_response(response)) {
      assert(response.status == LiveResponseStatus::Activated);
      active.plan = retained.back();
      generation = response.generation;
      ++updates;
    }
    const double t = tick * .01;
    if (tick && tick % 5 == 0) {
      assert(observe(t));
      const auto at = tick + 3;
      auto replacement =
          follower.plan(state(active.plan->command(at)), 10, .01, {generation + 1, generation, at, 7}, 40,
                        history, frames, stamp(10 + t));
      if (!replacement) {
        std::cerr << name << " update " << tick << ": " << replacement.diagnostic << '\n';
        std::abort();
      }
      retained.push_back(replacement.plan);
      assert(box.try_offer(*replacement.plan));
    }
    assert(execution.poll(stamp(10 + schedule_detail::tick_time(tick, .01)), output) ==
           ExecutionStatus::Running);
    const auto current = state(output.joints);
    const auto pose = robot.kinematics->fk(current.q);
    auto truth = origin;
    truth.position[0] += displacement(t);
    error = std::max(error, kinematic_detail::norm(kinematic_detail::sub(pose.position, truth.position)));
    assert(rotation_distance(pose.orientation, truth.orientation) < .0001);
    const auto d = robot.kinematics->differentiate(current.q, current.dq, current.ddq, Vector(6));
    assert(d.velocity.data[0] > .001);
  }
  assert(updates == 29 && error < .0001);
  LivePlanResponse response;
  if (box.try_response(response))
    active.plan = retained.back();
  for (; tick <= active.plan->end_tick(); ++tick) {
    const auto status = execution.poll(stamp(10 + schedule_detail::tick_time(tick, .01)), output);
    assert(status ==
           (tick == active.plan->end_tick() ? ExecutionStatus::Finished : ExecutionStatus::Running));
  }
  assert(buffer_detail::rest(output.joints));
  std::cout << name << ": " << updates << " overlapping updates, nominal error " << error
            << " m; lost-update fallback reached rest\n";
}
#endif
} // namespace
int main() {
  TrackingFrameSnapshot frames("world", {}, 1, stamp(0), stamp(100));
  ConveyorObservationHistory history("part", 1, {"world"});
  const ObjectObservation object{"part", "world", {{.1, 0, 0}, {1, 0, 0, 0}}, stamp(10), stamp(10),
                                 .00001, 0};
  const EncoderObservation encoder{0, .04, 0, stamp(10), stamp(10), 0, 0};
  assert(history.ingest(object, encoder, frames, stamp(10)));
  assert(history.ingest(object, encoder, frames, stamp(10)).status == ObservationUpdateStatus::OutOfOrder);
  auto outlier = object;
  outlier.measured_at = stamp(10.05);
  outlier.received_at = stamp(10.05);
  outlier.pose.position[0] = 10;
  auto next = encoder;
  next.position_m = .002;
  next.measured_at = stamp(10.05);
  next.received_at = stamp(10.05);
  assert(history.ingest(outlier, next, frames, stamp(10.05)).status == ObservationUpdateStatus::Outlier);
  assert(history.size() == 1);
  ConveyorObservationHistory scaled_history("part", 1, {"world", {1, 0, 0}, 2});
  ConveyorObservationHistory plain_history("part", 1, {"world"});
  assert(scaled_history.ingest(object, encoder, frames, stamp(10)));
  assert(plain_history.ingest(object, encoder, frames, stamp(10)));
  const auto scaled_prediction = scaled_history.predict(frames, stamp(10), stamp(10.4));
  const auto plain_prediction = plain_history.predict(frames, stamp(10), stamp(10.4));
  assert(scaled_prediction && plain_prediction);
  assert(scaled_prediction.target->velocity_error_m_s >= 1.99 * plain_prediction.target->velocity_error_m_s);

  const JointLimits limits{Vector(3, -2), Vector(3, 2), Vector(3, 1), Vector(3, 3), Vector(3, 30)};
  GantryConveyorFollower follower(limits, {.8, 2, 20, 1, 1, 1});
  const auto plan = follower.plan({{.1, 0, 0}, {.04, 0, 0}, Vector(3), 10}, 10, .01, {1, 0, 0, 7}, 40,
                                  history, frames, stamp(10));
  assert(plan);
  LivePlanMailbox<3> box;
  LiveMotionExecutor<3> executor(*plan.plan, box, {0, .001, 0, 0});
  LiveJointCommand<3> output;
  TrackingFrameSnapshot recalibrated("world", {}, 2, stamp(0), stamp(100));
  for (std::uint64_t tick = 0; tick <= plan.plan->end_tick(); ++tick) {
    if (tick == 16 || tick == 25) {
      const auto now = stamp(10 + schedule_detail::tick_time(tick, .01));
      const auto &snapshot = tick == 25 ? recalibrated : frames;
      const auto prediction = history.predict(snapshot, now, {now.seconds + .1, 7, 0});
      assert(prediction.status ==
             (tick == 25 ? PredictionStatus::CalibrationUnavailable : PredictionStatus::StaleObservation));
      const auto at = tick + 3;
      const auto c = plan.plan->command(at);
      const auto replacement =
          follower.plan({Vector(c.q.begin(), c.q.end()), Vector(c.dq.begin(), c.dq.end()),
                         Vector(c.ddq.begin(), c.ddq.end()), c.timestamp_s},
                        10, .01, {2, 1, at, 7}, 40, history, snapshot, now);
      assert(!replacement && !replacement.plan && !replacement.diagnostic.empty());
      if (tick == 25)
        assert(history.ingest(object, encoder, recalibrated, now).status ==
               ObservationUpdateStatus::CalibrationChanged);
      assert(history.size() == 1 && executor.generation() == 1);
    }
    const auto status = executor.poll(stamp(10 + schedule_detail::tick_time(tick, .01)), output);
    assert(status == (tick == plan.plan->end_tick() ? ExecutionStatus::Finished : ExecutionStatus::Running));
    assert(output.joints.tick == tick && output.generation == 1);
    if (tick <= 40)
      assert(std::abs(output.joints.q[0] - (.1 + .04 * tick * .01)) < 1e-10);
  }
  assert(buffer_detail::rest(output.joints));
  const auto saved = output;
  assert(executor.poll(stamp(20), output) == ExecutionStatus::Inactive);
  assert(output.joints == saved.joints && output.generation == saved.generation);
  history.reset("part", 2);
  assert(history.size() == 0 && !history.predict(recalibrated, stamp(10), stamp(10.1)));
  assert(history.ingest(object, encoder, recalibrated, stamp(10)));
  assert(history.predict(recalibrated, stamp(10), stamp(10.1)));
  assert(!history.predict(frames, stamp(11), stamp(11.4)));
#ifdef MOTION_WITH_SERIAL
  for (const auto *name : {"ur5e", "abb_irb120", "fanuc_lrmate200id"})
    serial_follow(name);
#endif
  std::cout << "History association/outlier/stale gates, moving following and admitted fallback passed\n";
}
