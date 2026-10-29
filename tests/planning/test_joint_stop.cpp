#include "motion/execution/boundary_polynomial_schedule.hpp"
#include "motion/execution/stop_schedules.hpp"
#include "motion/planning/ruckig_joint_generator.hpp"
#include <cassert>
#include <iostream>
#include <random>
using namespace motion;
int main() {
  const JointLimits limits{{-10}, {10}, {3}, {2}, {4}};
  JointStopPlanner planner(limits);
  for (double v : {-.5, .5, -2., 2.}) {
    auto p = planner.plan({{0}, {v}, {0}, 10});
    assert(p);
    const double speed = std::abs(v), T = speed < 1 ? 2 * std::sqrt(speed / 4) : speed / 2 + .5;
    assert(std::abs(p.trajectory->duration() - T) < 1e-10);
    assert(std::abs(p.trajectory->terminal_state().q[0] - v * T / 2) < 1e-10);
    for (int segment = 0; segment < 13; ++segment) {
      const double lo = T * (segment / 13.), hi = T * ((segment + 1) / 13.);
      const auto box = p.trajectory->enclose_state(lo, hi);
      for (int sample = 0; sample <= 20; ++sample) {
        const auto state = p.trajectory->sample_motion(sample == 20 ? hi : lo + (hi - lo) * (sample / 20.));
        const double values[4] = {state.joints.q[0], state.joints.dq[0], state.joints.ddq[0], state.jerk[0]};
        for (int o = 0; o < 4; ++o)
          assert(values[o] >= box[0][o].first - 1e-12 && values[o] <= box[0][o].second + 1e-12);
      }
    }
    auto grid = schedule_joint_stop(p.trajectory, .013);
    assert(grid && grid.program->duration() >= T && grid.program->duration() - T < .013 + 1e-12);
    assert(is_rest(grid.program->sample_tick(grid.program->total_ticks()), 0));
  }
  auto rest = planner.plan({{1}, {0}, {0}, 10});
  assert(rest && rest.status == StopPlanStatus::AlreadyStopped);
  auto outward = JointStopPlanner({{-1}, {1}, {3}, {2}, {4}}).plan({{1}, {.1}, {0}, 10});
  assert(!outward && outward.status == StopPlanStatus::CandidateRejected);
  auto vpeak = JointStopPlanner({{-10}, {10}, {1}, {2}, {2}}).plan({{0}, {.95}, {1}, 10});
  assert(!vpeak);
  std::mt19937 random(24);
  std::uniform_real_distribution<double> v(-2, 2), a(-1.5, 1.5);
  for (int k = 0; k < 300; ++k) {
    auto candidate = planner.plan({{0}, {v(random)}, {a(random)}, 10});
    if (candidate) {
      for (int sample = 0; sample <= 100; ++sample) {
        auto state = candidate.trajectory->sample(candidate.trajectory->duration() * (sample / 100.));
        assert(within_joint_limits(state, limits));
      }
      assert(is_rest(candidate.trajectory->terminal_state(), 0));
    }
  }
  auto polynomial = plan_boundary_quintic(limits, {{0}, {0}, {0}, 10}, {{1}, {0}, {0}}, 3);
  assert(polynomial);
  auto source = std::make_shared<BoundaryPolynomialSchedule>(polynomial.trajectory, .01);
  const auto transition = prepare_stop_transition(source, 100);
  assert(transition);
  for (std::uint64_t k = 0; k <= 100; ++k) {
    const auto x = transition.program->sample_tick(k), y = source->sample_tick(k);
    assert(x.q == y.q && x.dq == y.dq && x.ddq == y.ddq && x.timestamp_s == y.timestamp_s);
  }
  assert(is_rest(transition.program->sample_tick(transition.program->total_ticks()), 0));
  OnlineJointPlanner online(limits, .01);
  assert(online.reset({{0}, {.2}, {0}, 10}) == StepStatus::Finished);
  assert(online.retarget({{1}, {.1}, {0}}) == StepStatus::Running);
  JointState output{{9}, {9}, {9}, 99};
  for (int k = 0; k < 10000; ++k) {
    const auto status = online.step(output);
    assert(status == StepStatus::Running || status == StepStatus::Finished);
    assert(within_joint_limits(output, limits));
    if (status == StepStatus::Finished)
      break;
  }
  assert(std::abs(output.q[0] - 1) < 1e-8 && std::abs(output.dq[0] - .1) < 1e-8 && output.timestamp_s > 10);
  auto saved = output;
  assert(online.step(output) == StepStatus::InvalidInput && output.q == saved.q &&
         output.timestamp_s == saved.timestamp_s);
  assert(online.retarget({{-1}, {0}, {0}}) == StepStatus::Running);
  assert(online.retarget({{20}, {0}, {0}}) == StepStatus::InvalidInput);
  assert(online.step(output) == StepStatus::Running);
  std::cout << "Closed-form stop oracles, 300 moving starts, phase enclosures, interior limit rejection, "
               "prepared transition and moving online retarget passed\n";
}
