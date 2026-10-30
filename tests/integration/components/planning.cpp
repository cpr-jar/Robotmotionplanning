#include "motion/planning/cartesian_spline.hpp"
#include "motion/planning/path_time_parameterization.hpp"
#include <cassert>
int main() {
  auto geometry =
      std::make_shared<motion::CartesianLinePath>(motion::Pose{}, motion::Pose{{.01, 0, 0}, {1, 0, 0, 0}});
  auto path = motion::map_cartesian_spline(
      std::make_shared<motion::CartesianGantry>(),
      std::make_shared<motion::PoseSweepPath>(geometry, std::array<double, 4>{1, 0, 0, 0}), {0, 0, 0});
  assert(path);
  const motion::JointLimits limits{motion::Vector(3, -1), motion::Vector(3, 1), motion::Vector(3, 1),
                                   motion::Vector(3, 2), motion::Vector(3, 10)};
  const auto plan = motion::PathTimeParameterizer(limits).plan(
      path.path, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 0}, {1, 2, 10, 1, 2, 10});
  assert(plan && std::abs(plan.trajectory->sample(plan.trajectory->duration()).q[0] - .01) < 1e-10);
}
