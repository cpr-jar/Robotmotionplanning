#include "motion/planning/joint_trajectory.hpp"
#include "motion/planning/online_joint_generator.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace motion;
int main() {
  JointLimits l{{-2, -2, -2}, {2, 2, 2}, {0.8, 0.8, 0.8}, {1.5, 1.5, 1.5}, {5, 5, 5}};
  JointState start{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 10};
  JointTarget goal{{1, 0.8, -0.7}, {0, 0, 0}, {0, 0, 0}};
  QuinticJointPlanner jp(l);
  auto p = jp.plan(start, goal);
  assert(p && p.trajectory);
  auto segment = dynamic_cast<const QuinticRestTrajectory *>(p.trajectory.get());
  assert(segment);
  for (int k = 0; k <= 10000; ++k) {
    auto s = p.trajectory->sample(p.trajectory->duration() * k / 10000.0);
    assert(within_joint_limits(s, l, 1e-7));
  }
  for (std::size_t i = 0; i < 3; ++i)
    assert(segment->peak_jerk(i) <= l.max_jerk[i] + 1e-8);
  auto invalid = jp.plan(start, {{3, 0, 0}, {0, 0, 0}, {0, 0, 0}});
  assert(!invalid && invalid.status == PlanStatus::Infeasible);
  auto moving = start;
  moving.dq[0] = 0.1;
  assert(!jp.plan(moving, goal));
  CartesianGantry gantry;
  CartesianLimits c{0.5, 0.7, 2.0, 1, 1, 1};
  GantryLinearPlanner lin(l, gantry);
  CartesianRequest request{{{0, 0, 0}, {1, 0, 0, 0}}, {{1, 0.8, -0.7}, {1, 0, 0, 0}}, c};
  auto cart = lin.plan(request, start);
  assert(cart);
  const double length = std::sqrt(1 + 0.8 * 0.8 + 0.7 * 0.7);
  const double T = cart.trajectory->duration();
  assert(1.875 * length / T <= c.max_linear_velocity + 1e-8);
  assert((10 / std::sqrt(3.0)) * length / (T * T) <= c.max_linear_acceleration + 1e-8);
  assert(60 * length / (T * T * T) <= c.max_linear_jerk + 1e-8);
  auto end = cart.trajectory->sample(T);
  for (std::size_t i = 0; i < 3; ++i)
    assert(std::abs(end.q[i] - goal.q[i]) < 1e-8);
  RestOnlyOnlineGenerator online(l, 0.005);
  assert(online.reset(start) == StepStatus::Finished);
  assert(online.retarget(goal) == StepStatus::Running);
  JointState next;
  assert(online.step(next) == StepStatus::Running);
  assert(online.retarget(goal) == StepStatus::InvalidInput);
  std::cout << "Trajectory checks passed\n";
}
