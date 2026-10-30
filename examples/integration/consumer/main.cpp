#include "motion/core/clock_time.hpp"
#include "motion/execution/boundary_polynomial_schedule.hpp"
#include "motion/execution/live_motion_session.hpp"
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
  const auto boundary = motion::plan_boundary_quintic(limits, {{0}, {0}, {0}, 0}, {{0}, {0}, {0}}, .02);
  if (!boundary)
    return 1;
  auto cached = motion::prepare_live_plan<1>(
      std::make_shared<motion::BoundaryPolynomialSchedule>(boundary.trajectory, .01), 0, {1, 0, 0, 7});
  if (!cached)
    return 1;
  motion::LiveMotionSession<1> session(std::move(cached.plan), {0, .001, 0, 0});
  struct Sink {
    bool try_write(const motion::LiveJointCommand<1> &) noexcept { return true; }
    void request_stop() noexcept {}
  } sink;
  if (session.dispatch({0, 7, 0}, sink) != motion::ExecutionStatus::Running)
    return 1;
  session.plans().request_shutdown();
  if (session.dispatch({.01, 7, 0}, sink) != motion::ExecutionStatus::Inactive ||
      !session.plans().release_after_shutdown())
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
