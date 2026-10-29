#include "motion/cartesian_spline.hpp"
#include "motion/path_timing.hpp"
#include "motion/reference_kinematics.hpp"
#include <cassert>
#include <iostream>
using namespace motion;
void check(const IDifferentialKinematics &model, const Vector &q) {
  const auto pose = model.fk(q);
  auto seed = q;
  for (auto &x : seed)
    x += .01;
  auto inverse = model.ik(pose, seed);
  assert(inverse);
  auto recovered = model.fk(*inverse);
  assert(kinematic_detail::norm(kinematic_detail::sub(pose.position, recovered.position)) < 1e-8);
  Vector v(q.size(), .1), a(q.size(), .03), j(q.size(), -.02);
  const double h = 1e-4;
  auto qp = q, qm = q, vp = v, vm = v, ap = a, am = a;
  for (std::size_t i = 0; i < q.size(); ++i) {
    qp[i] += v[i] * h + a[i] * h * h / 2 + j[i] * h * h * h / 6;
    qm[i] -= v[i] * h - a[i] * h * h / 2 + j[i] * h * h * h / 6;
    vp[i] += a[i] * h + j[i] * h * h / 2;
    vm[i] -= a[i] * h - j[i] * h * h / 2;
    ap[i] += j[i] * h;
    am[i] -= j[i] * h;
  }
  const auto center = model.differentiate(q, v, a, j), plus = model.differentiate(qp, vp, ap, j),
             minus = model.differentiate(qm, vm, am, j);
  for (int i = 0; i < 6; ++i) {
    assert(std::abs((plus.velocity.data[i] - minus.velocity.data[i]) / (2 * h) -
                    center.acceleration.data[i]) < 1e-7);
    assert(std::abs((plus.acceleration.data[i] - minus.acceleration.data[i]) / (2 * h) -
                    center.jerk.data[i]) < 1e-7);
  }
}
int main() {
  ScaraKinematics scara({.3, .2}, {{-3, -3, -1, -6}, {3, 3, 1, 6}});
  check(scara, {.3, .7, .1, -.2});
  DeltaKinematics delta({.25, .05, .2, .5}, {Vector(3, -1.4), Vector(3, 1.4)});
  check(delta, {.3, .4, .2});
  const auto center = delta.fk({0, 0, 0});
  assert(std::abs(center.position[0]) < 1e-12 && std::abs(center.position[1]) < 1e-12);
  assert(std::abs(center.position[2] + .3) < 1e-12);
  for (int k = 0; k < 30; ++k)
    check(delta, {.2 + k * .002, .3, .1});

  auto delta_model = std::make_shared<DeltaKinematics>(delta);
  const Vector seed{.3, .4, .2};
  auto first = delta.fk(seed), goal = first;
  goal.position[0] += .003;
  const auto desired =
      std::make_shared<PoseSweepPath>(std::make_shared<CartesianLinePath>(first, goal), goal.orientation);
  const auto line = map_cartesian_spline(delta_model, desired, seed);
  assert(line);
  auto arc_center = first.position;
  arc_center[0] -= .003;
  const auto circle_desired = std::make_shared<PoseSweepPath>(
      std::make_shared<CartesianArcPath>(arc_center, first, PathVector3{0, 0, 1}, .5), first.orientation);
  const auto circle = map_cartesian_spline(delta_model, circle_desired, seed);
  assert(circle);
  for (const auto &pair :
       std::vector<std::pair<std::shared_ptr<const CartesianSplinePath>, std::shared_ptr<const IPosePath>>>{
           {line.path, desired}, {circle.path, circle_desired}}) {
    for (int k = 0; k <= 100; ++k) {
      auto actual = pair.first->evaluate(k / 100.).tcp.pose, target = pair.second->evaluate(k / 100.);
      assert(kinematic_detail::norm(kinematic_detail::sub(actual.position, target.position)) <= .0002);
    }
    const auto timed =
        PathTimeParameterizer({Vector(3, -1.4), Vector(3, 1.4), Vector(3, 1), Vector(3, 3), Vector(3, 10)})
            .plan(pair.first, {seed, Vector(3), Vector(3), 10}, {.3, .5, 2, .3, .5, 2});
    assert(timed);
  }
  auto blend = std::make_shared<BlendedPosePath>(
      std::vector<Pose>{{{0, 0, 0}, {1, 0, 0, 0}}, {{.1, 0, 0}, {1, 0, 0, 0}}, {{.1, .1, 0}, {1, 0, 0, 0}}},
      .002);
  auto blended = map_cartesian_spline(std::make_shared<CartesianGantry>(), blend, {0, 0, 0});
  assert(blended);
  const auto mid = blended.path->evaluate(.5);
  assert(std::hypot(mid.q_s[0], mid.q_s[1]) > 1e-4);
  std::cout << "SCARA and lower-assembly Delta FK/IK, independent symmetric sphere fixture and differential "
               "chain-rule checks passed\n";
}
