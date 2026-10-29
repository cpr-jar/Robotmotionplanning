#include "motion/io/robot_model_import.hpp"
#include "motion/planning/cartesian_spline.hpp"
#include "motion/planning/path_time_parameterization.hpp"
#include <cassert>
#include <iostream>
using namespace motion;
namespace {
void check_path(const std::shared_ptr<const IPosePath> &desired, const Vector &seed) {
  auto mapped = map_cartesian_spline(std::make_shared<CartesianGantry>(), desired, seed);
  if (!mapped) {
    std::cerr << mapped.diagnostic << '\n';
    std::abort();
  }
  const JointLimits limits{Vector(3, -1), Vector(3, 1), Vector(3, .2), Vector(3, .5), Vector(3, 2)};
  const CartesianLimits caps{.1, .3, 1, .2, .3, 1};
  auto planned = PathTimeParameterizer(limits).plan(mapped.path, {seed, Vector(3), Vector(3), 0}, caps);
  if (!planned) {
    std::cerr << planned.diagnostic << '\n';
    std::abort();
  }
  auto timed = std::dynamic_pointer_cast<const TimedPathTrajectory>(planned.trajectory);
  assert(timed);
  for (int k = 0; k <= 300; ++k) {
    auto point = timed->sample_motion(timed->duration() * (k / 300.));
    assert(within_joint_limits(point.joints, limits));
    for (int i = 0; i < 3; ++i)
      assert(std::abs(point.joint_jerk[i]) <= 2 + 1e-8);
    const Twist ds[3] = {point.tcp_derivatives.velocity, point.tcp_derivatives.acceleration,
                         point.tcp_derivatives.jerk};
    const double lc[3] = {.1, .3, 1};
    for (int order = 0; order < 3; ++order)
      assert(std::hypot(ds[order].data[0], ds[order].data[1], ds[order].data[2]) <= lc[order] + 1e-8);
    const double t = k / 300., s = t * t * t * (10 + t * (-15 + 6 * t));
    assert(kinematic_detail::norm(kinematic_detail::sub(point.tcp.position, desired->evaluate(s).position)) <=
           .0002 + 1e-10);
  }
  assert(is_rest(timed->sample(0), 0) && is_rest(timed->sample(timed->duration()), 0));
}
} // namespace
int main() {
  SerialChainKinematics turn({{"turn",
                               "base",
                               "tip",
                               SerialJointType::Revolute,
                               {},
                               {0, 0, 1},
                               -4 * kinematic_detail::pi,
                               4 * kinematic_detail::pi,
                               1}},
                             {}, {{1, 0, 0}, {1, 0, 0, 0}});
  for (double eps : {-1e-8, 0., 1e-8}) {
    const double angle = kinematic_detail::pi + eps;
    auto goal = turn.fk({angle});
    for (int sign : {-1, 1}) {
      auto signed_goal = goal;
      for (auto &x : signed_goal.orientation)
        x *= sign;
      auto result = turn.ik_detailed(signed_goal, {kinematic_detail::pi - .02});
      assert(result);
      assert(std::abs(result.solution[0] - angle) < 1e-6);
      assert(rotation_distance(turn.fk(result.solution).orientation, goal.orientation) < 1e-7);
    }
  }
  for (double length : {0., 1e-12, 1e-9, 1e-6}) {
    Pose start{{.1, 0, 0}, {1, 0, 0, 0}}, goal = start;
    goal.position[0] += length;
    check_path(
        std::make_shared<PoseSweepPath>(std::make_shared<CartesianLinePath>(start, goal), goal.orientation),
        {.1, 0, 0});
  }
  for (double sweep : {-2 * kinematic_detail::pi, 2 * kinematic_detail::pi}) {
    Pose first{{.02, 0, 0}, {1, 0, 0, 0}};
    auto arc = std::make_shared<CartesianArcPath>(PathVector3{0, 0, 0}, first, PathVector3{0, 0, 1}, sweep);
    assert(kinematic_detail::norm(kinematic_detail::sub(arc->evaluate(1).pose.position, first.position)) <
           1e-12);
    check_path(std::make_shared<PoseSweepPath>(arc, first.orientation), {.02, 0, 0});
  }
  // A Cartesian path beyond a configured joint limit must not become a plan.
  {
    Pose start{{.99, 0, 0}, {1, 0, 0, 0}}, goal{{1.01, 0, 0}, {1, 0, 0, 0}};
    auto desired =
        std::make_shared<PoseSweepPath>(std::make_shared<CartesianLinePath>(start, goal), goal.orientation);
    auto mapped = map_cartesian_spline(std::make_shared<CartesianGantry>(), desired, {.99, 0, 0});
    assert(mapped);
    const JointLimits limits{Vector(3, -1), Vector(3, 1), Vector(3, .2), Vector(3, .5), Vector(3, 2)};
    auto result = PathTimeParameterizer(limits).plan(mapped.path, {{.99, 0, 0}, Vector(3), Vector(3), 0},
                                                     {.1, .3, 1, .2, .3, 1});
    assert(!result && !result.trajectory && !result.diagnostic.empty());
  }
  std::size_t admitted = 0, rejected = 0;
  for (const char *name : {"ur5e", "abb_irb120", "fanuc_lrmate200id"}) {
    const auto directory = std::string(MOTION_SOURCE_DIR) + "/models/" + name;
    auto robot = load_serial_robot_files(directory + "/robot.urdf", directory + "/profile.yaml");
    std::vector<Vector> states{{.2, -.6, .7, .3, .8, -.2}, Vector(6)};
    for (double epsilon : {-1e-3, 1e-3, -1e-6, 1e-6, -1e-9, 1e-9, -1e-12, 1e-12}) {
      states.push_back({.2, -.6, .7, .3, epsilon, -.2});
      states.push_back(Vector(6, epsilon));
    }
    for (int axis = 0; axis < 6; ++axis)
      for (bool upper : {false, true}) {
        Vector q{.2, -.6, .7, .3, .8, -.2};
        q[axis] = (upper ? robot.joint_limits.max_position[axis] : robot.joint_limits.min_position[axis]) +
                  (upper ? -1 : 1) * 1e-10;
        states.push_back(q);
      }
    for (const auto &q : states) {
      const auto condition = robot.kinematics->singularity(q);
      assert(std::isfinite(condition.rank_margin) && condition.rank_margin >= 0);
      const auto p = robot.kinematics->fk(q);
      for (double angle :
           {0., kinematic_detail::pi - 1e-9, kinematic_detail::pi, kinematic_detail::pi + 1e-9}) {
        Pose goal = p;
        if (angle != 0)
          goal.orientation =
              tracking_detail::multiply({std::cos(angle / 2), 0, 0, std::sin(angle / 2)}, p.orientation);
        auto result = robot.kinematics->ik_detailed(goal, q);
        assert(result.iterations < robot.kinematics->solver_options().max_iterations);
        if (result) {
          ++admitted;
          assert(within_joint_limits({result.solution, Vector(6), Vector(6), 0}, robot.joint_limits, 0));
          const auto actual = robot.kinematics->fk(result.solution);
          assert(rotation_distance(actual.orientation, goal.orientation) <= 1e-7 + 1e-12);
          assert(kinematic_detail::norm(kinematic_detail::sub(actual.position, goal.position)) <=
                 1e-7 + 1e-12);
        } else {
          ++rejected;
          assert(!result.diagnostic.empty());
        }
      }
    }
    auto beyond = robot.joint_limits.max_position;
    beyond[0] = std::nextafter(beyond[0], INFINITY);
    bool failed = false;
    try {
      robot.kinematics->fk(beyond);
    } catch (const std::invalid_argument &) {
      failed = true;
    }
    assert(failed);
  }
  assert(admitted && rejected);
  std::cout << "Kinematic edge targets admitted=" << admitted << ", explicitly rejected=" << rejected
            << "; wrap, antipodes, half-turn, tiny lines and full circles checked\n";
}
