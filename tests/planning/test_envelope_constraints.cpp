#include "motion/planning/envelope_constraints.hpp"
#include <cassert>
#include <iostream>
using namespace motion;
int main() {
  using enclosure::Interval;
  const auto s = enclosure::sin(Interval(-2, 2));
  assert(s.lo <= -1 && s.hi >= 1);
  auto product = Interval(-2, 3) * Interval(-4, 5);
  assert(product.lo <= -12 && product.hi >= 15);
  JointLimits limits{{-10}, {10}, {10}, {10}, {10}};
  auto p = plan_boundary_quintic(limits, {{0}, {0}, {0}, 10}, {{1}, {0}, {0}}, 2);
  assert(p);
  for (int k = 0; k <= 100; ++k) {
    const double t = k / 50.;
    const auto [state, jerk] = p.trajectory->sample_with_jerk(t);
    const double u = t / 2;
    assert(std::abs(state.q[0] - (10 * u * u * u - 15 * u * u * u * u + 6 * u * u * u * u * u)) < 1e-12);
    assert(std::abs(jerk[0] - (60 - 360 * u + 360 * u * u) / 8) < 1e-12);
  }
  auto too_fast = plan_boundary_quintic(limits, {{0}, {0}, {0}, 10}, {{1}, {0}, {0}}, .1);
  assert(!too_fast);
  auto zero_duration = plan_boundary_quintic(limits, {{0}, {0}, {0}, 10}, {{1}, {0}, {0}}, 0);
  assert(!zero_duration);
  const JointLimits xyz{Vector(3, -2), Vector(3, 2), Vector(3, 2), Vector(3, 5), Vector(3, 50)};
  const auto move = plan_boundary_quintic(xyz, {{-.5, 0, 0}, Vector(3), Vector(3), 10},
                                          {{.5, 0, 0}, Vector(3), Vector(3)}, 2);
  assert(move);
  CartesianGantry model;
  const CartesianLimits caps{2, 5, 50, 1, 1, 1};
  assert(check_polynomial_constraints(model, *move.trajectory, caps));
  auto obstacle = std::make_shared<TcpObstacleConstraint>(
      std::vector<WorkspaceBox>{{{-.1, -.1, -.1}, {.1, .1, .1}}}, .01);
  assert(!check_polynomial_constraints(model, *move.trajectory, caps, {obstacle}));
  assert(!check_polynomial_constraints(model, *move.trajectory, {.01, 5, 50, 1, 1, 1}));
  auto spline = map_cartesian_spline(
      std::make_shared<CartesianGantry>(),
      std::make_shared<PoseSweepPath>(
          std::make_shared<CartesianLinePath>(Pose{{0, 0, 0}, {1, 0, 0, 0}}, Pose{{.1, .1, 0}, {1, 0, 0, 0}}),
          std::array<double, 4>{1, 0, 0, 0}),
      {0, 0, 0});
  assert(spline);
  const auto timed = PathTimeParameterizer(xyz).plan(spline.path, {{0, 0, 0}, Vector(3), Vector(3), 0}, caps);
  assert(timed);
  for (int k = 0; k <= 100; ++k) {
    const auto state = timed.trajectory->sample(timed.trajectory->duration() * (k / 100.));
    assert(within_joint_limits(state, xyz));
    assert(std::abs(state.q[0] - state.q[1]) < 1e-12);
  }
  std::cout << "Independent polynomial oracle, interval primitives, interior obstacle rejection and "
               "Cartesian spline limits passed\n";
}
