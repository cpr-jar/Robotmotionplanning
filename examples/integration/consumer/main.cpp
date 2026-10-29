#include "motion/core/clock_time.hpp"
#include "motion/planning/joint_trajectory.hpp"
#ifdef MOTION_WITH_SERIAL
#include "motion/program/robot_program.hpp"
#endif
#include <iostream>
int main() {
  const motion::JointLimits limits{{-2}, {2}, {1}, {2}, {10}};
  const motion::QuinticJointPlanner joint_planner(limits);
  const auto planned_motion = joint_planner.plan({{0}, {0}, {0}, 0}, {{.1}, {0}, {0}});
  if (!planned_motion)
    return 1;
#ifdef MOTION_WITH_SERIAL
  const auto program_source =
      motion::parse_robot_program_yaml("schema_version: 1\nunits: SI\nrobot_profile: example.yaml\n"
                                       "start: {joints: [0]}\nsteps: [{type: WAIT, seconds: 0.01}]\n");
  if (program_source.instructions().size() != 1)
    return 1;
#endif
  std::cout << "External CMake consumer: library link and public API passed\n";
}
