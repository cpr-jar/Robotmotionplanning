#include "motion/kinematics/serial_chain_kinematics.hpp"
#include "motion/planning/cartesian_spline.hpp"
#include "motion/planning/path_time_parameterization.hpp"
#include <cassert>
int main() {
  auto model = std::make_shared<motion::SerialChainKinematics>(std::vector<motion::SerialJoint>{
      {"x", "base", "tip", motion::SerialJointType::Prismatic, {}, {1, 0, 0}, -2, 2, 1}});
  auto geometry = std::make_shared<motion::CartesianLinePath>(model->fk({.2}), model->fk({.21}));
  auto mapped = motion::map_cartesian_spline(
      model, std::make_shared<motion::PoseSweepPath>(geometry, std::array<double, 4>{1, 0, 0, 0}), {.2});
  assert(mapped);
  const auto planned = motion::PathTimeParameterizer({{-2}, {2}, {1}, {2}, {10}})
                           .plan(mapped.path, {{.2}, {0}, {0}, 0}, {1, 2, 10, 1, 2, 10});
  assert(planned && std::abs(planned.trajectory->sample(planned.trajectory->duration()).q[0] - .21) < 1e-7);
}
