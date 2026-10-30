#include "motion/execution/live_motion_executor.hpp"
#include "motion/planning/cartesian_spline.hpp"
#include "motion/planning/path_time_parameterization.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using namespace motion;
namespace {
constexpr double invalid_number = std::numeric_limits<double>::quiet_NaN();
const JointLimits limits{{-2}, {2}, {2}, {5}, {50}};
struct TestSchedule final : IExecutionSchedule {
  JointState initial{{0}, {0}, {0}, 10};
  JointLimits caps{limits};
  std::vector<JointState> states{3, initial};
  std::vector<std::uint64_t> boundaries{2};
  double span{.02}, cycle{.01}, time_offset{};
  TestSchedule() {
    for (std::size_t i = 0; i < states.size(); ++i)
      states[i].timestamp_s = 10 + schedule_detail::tick_time(i, cycle);
  }
  double duration() const noexcept override { return span; }
  double cycle_s() const noexcept override { return cycle; }
  std::uint64_t total_ticks() const noexcept override { return 2; }
  double tick_time(std::uint64_t tick) const noexcept override {
    return schedule_detail::tick_time(tick, cycle) + time_offset;
  }
  const std::vector<std::uint64_t> &boundary_ticks() const noexcept override { return boundaries; }
  const JointState &initial_state() const noexcept override { return initial; }
  const JointLimits &joint_limits() const noexcept override { return caps; }
  JointState sample(double) const override { return initial; }
  JointState sample_tick(std::uint64_t tick) const override { return states.at(tick); }
};
void live_providers() {
  auto good = std::make_shared<TestSchedule>();
  const auto ready = prepare_live_plan<1>(good, 10, {1, 0, 0, 7});
  assert(ready);
  for (std::uint64_t tick = 0; tick <= 2; ++tick) {
    const auto &command = ready.plan->command(tick);
    assert(command.q[0] == 0 && command.dq[0] == 0 && command.ddq[0] == 0);
    assert(command.tick == tick && command.timestamp_s == good->states[tick].timestamp_s);
    assert(command.terminal == (tick == 2));
  }
  const auto reject = [](auto change) {
    auto schedule = std::make_shared<TestSchedule>();
    change(*schedule);
    const auto result = prepare_live_plan<1>(schedule, 10, {1, 0, 0, 7});
    assert(!result && !result.plan && !result.diagnostic.empty());
  };
  for (auto member : {&JointState::dq, &JointState::ddq, &JointState::q})
    for (std::size_t size : {0u, 2u, 64u}) {
      reject([&](auto &s) { (s.initial.*member).resize(size); });
      reject([&](auto &s) { (s.states[1].*member).resize(size); });
    }
  for (auto member : {&JointLimits::min_position, &JointLimits::max_position, &JointLimits::max_velocity,
                      &JointLimits::max_acceleration, &JointLimits::max_jerk}) {
    reject([&](auto &s) { (s.caps.*member).resize(2); });
    reject([&](auto &s) { (s.caps.*member)[0] = invalid_number; });
  }
  // The review reproducer: limits and sampled states agree with each other,
  // but have more axes than the fixed command and the initial state's q.
  reject([](auto &s) {
    s.caps = {Vector(64, -2), Vector(64, 2), Vector(64, 2), Vector(64, 5), Vector(64, 50)};
    for (auto &state : s.states)
      state = {Vector(64), Vector(64), Vector(64), state.timestamp_s};
  });
  reject([](auto &s) { s.initial.q[0] = invalid_number; });
  reject([](auto &s) { s.initial.q[0] = 3; });
  reject([](auto &s) { s.states[1].dq[0] = invalid_number; });
  reject([](auto &s) { s.states[1].q[0] = 3; });
  reject([](auto &s) { s.states[0].q[0] = .1; });
  reject([](auto &s) { s.states[1].timestamp_s = invalid_number; });
  reject([](auto &s) { s.states[1].timestamp_s += .001; });
  reject([](auto &s) { s.cycle = invalid_number; });
  reject([](auto &s) { s.span = invalid_number; });
  reject([](auto &s) { s.span = .03; });
  reject([](auto &s) { s.time_offset = .001; });
  reject([](auto &s) { s.boundaries = {2, 1}; });
  reject([](auto &s) { s.boundaries = {3}; });
  reject([](auto &s) { s.states[2].dq[0] = .1; });
}
struct TestPath final : IBoundedJointPath {
  JointPathPoint initial{{0}, {0}, {0}, {0}, {}}, interior{initial}, last{initial};
  JointPathBounds envelope{{0}, {0}, {Vector{0}, Vector{0}, Vector{0}}, {}, {}};
  JointPathPoint evaluate(double s) const override { return s == 0 ? initial : s == 1 ? last : interior; }
  JointPathBounds bounds() const override { return envelope; }
};
void path_providers() {
  const PathTimeParameterizer planner(limits);
  const JointState start{{0}, {0}, {0}, 10};
  const CartesianLimits caps{1, 1, 1, 1, 1, 1};
  assert(planner.plan(std::make_shared<TestPath>(), start, caps));
  for (auto member : {&JointState::q, &JointState::dq, &JointState::ddq}) {
    auto bad_start = start;
    (bad_start.*member).clear();
    const auto result = planner.plan(std::make_shared<TestPath>(), bad_start, caps);
    assert(!result && result.status == PlanStatus::InvalidInput);
  }
  const auto reject = [&](auto change) {
    auto path = std::make_shared<TestPath>();
    change(*path);
    const auto result = planner.plan(path, start, caps);
    assert(!result && !result.trajectory && !result.diagnostic.empty());
  };
  for (auto member :
       {&JointPathPoint::q, &JointPathPoint::q_s, &JointPathPoint::q_ss, &JointPathPoint::q_sss}) {
    for (std::size_t size : {0u, 2u}) {
      reject([&](auto &p) { (p.initial.*member).resize(size); });
      reject([&](auto &p) { (p.last.*member).resize(size); });
    }
    reject([&](auto &p) { (p.initial.*member)[0] = invalid_number; });
  }
  reject([](auto &p) { p.envelope.lower[0] = invalid_number; });
  reject([](auto &p) { p.envelope.upper[0] = invalid_number; });
  reject([](auto &p) { p.envelope.lower[0] = 1; });
  reject([](auto &p) { p.envelope.upper.resize(2); });
  for (int order = 0; order < 3; ++order) {
    reject([&](auto &p) { p.envelope.joint_abs[order].resize(2); });
    reject([&](auto &p) { p.envelope.joint_abs[order][0] = invalid_number; });
    reject([&](auto &p) { p.envelope.joint_abs[order][0] = -1; });
  }
  reject([](auto &p) { p.initial.tcp.pose.position[0] = invalid_number; });
  reject([](auto &p) { p.initial.tcp.pose.orientation.fill(0); });
  reject([](auto &p) { p.last.tcp.angular_derivatives[2][1] = invalid_number; });
  // A provider may fail only in the interior: every sample must check before indexing.
  for (auto member :
       {&JointPathPoint::q, &JointPathPoint::q_s, &JointPathPoint::q_ss, &JointPathPoint::q_sss}) {
    auto path = std::make_shared<TestPath>();
    (path->interior.*member).clear();
    const auto plan = planner.plan(path, start, caps);
    assert(plan);
    bool rejected = false;
    try {
      plan.trajectory->sample(plan.trajectory->duration() / 2);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    assert(rejected);
  }
}
struct TestKinematics final : IDifferentialKinematics {
  KinematicJointLimits limits{Vector(3, -2), Vector(3, 2)};
  Vector solution{0, 0, 0};
  CartesianGantry gantry;
  std::size_t dof() const override { return 3; }
  Pose fk(const Vector &q) const override { return gantry.fk(q); }
  std::optional<Vector> ik(const Pose &, const Vector &) const override { return solution; }
  std::vector<Vector> jacobian(const Vector &q) const override { return gantry.jacobian(q); }
  const KinematicJointLimits &position_limits() const noexcept override { return limits; }
  SingularityInfo singularity(const Vector &) const override { return {}; }
  KinematicDerivatives differentiate(const Vector &, const Vector &, const Vector &,
                                     const Vector &) const override {
    return {};
  }
};
struct TestPosePath final : IPosePath {
  Pose first{}, other{};
  double speed{.01};
  Pose evaluate(double s) const override { return s == 0 ? first : other; }
  double linear_speed_bound() const noexcept override { return speed; }
  double angular_speed_bound() const noexcept override { return 0; }
};
void spline_providers() {
  const auto desired = std::make_shared<PoseSweepPath>(
      std::make_shared<CartesianLinePath>(Pose{}, Pose{{.01, 0, 0}, {1, 0, 0, 0}}),
      std::array<double, 4>{1, 0, 0, 0});
  const auto reject = [&](auto change) {
    auto model = std::make_shared<TestKinematics>();
    change(*model);
    const auto result = map_cartesian_spline(model, desired, {0, 0, 0});
    assert(!result && !result.path && result.status == PlanStatus::InvalidInput);
  };
  for (std::size_t size : {0u, 2u, 4u}) {
    reject([&](auto &m) { m.solution.resize(size); });
    reject([&](auto &m) { m.limits.lower.resize(size); });
    reject([&](auto &m) { m.limits.upper.resize(size); });
  }
  reject([](auto &m) { m.solution[0] = invalid_number; });
  reject([](auto &m) { m.limits.lower[0] = invalid_number; });
  reject([](auto &m) { m.limits.upper[0] = -3; });
  const auto reject_pose = [&](auto change) {
    auto path = std::make_shared<TestPosePath>();
    change(*path);
    const auto result = map_cartesian_spline(std::make_shared<CartesianGantry>(), path, {0, 0, 0});
    assert(!result && result.status == PlanStatus::InvalidInput);
  };
  reject_pose([](auto &p) { p.first.position[0] = invalid_number; });
  reject_pose([](auto &p) { p.other.position[0] = invalid_number; });
  reject_pose([](auto &p) { p.speed = invalid_number; });
  reject_pose([](auto &p) { p.speed = -1; });
}
} // namespace
int main() {
  live_providers();
  path_providers();
  spline_providers();
  std::cout << "Malformed schedule/path providers rejected before fixed-buffer copies or indexing\n";
}
