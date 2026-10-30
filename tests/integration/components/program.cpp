#include "motion/program/robot_program.hpp"
#include <cassert>
int main() {
  const auto program =
      motion::load_robot_program(std::string(MOTION_SOURCE_DIR) + "/programs/examples/ur5e_demo.yaml");
  const auto report = motion::simulate_robot_program(program);
  assert(report.commands > 100 && report.rest_boundaries == program.reference->segment_count());
}
