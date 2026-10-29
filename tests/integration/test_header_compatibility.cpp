// Both header layouts must remain usable in the same translation unit.
#include "motion/boundary_polynomial.hpp"
#include "motion/calibration.hpp"
#include "motion/calibration/rigid_calibration.hpp"
#include "motion/cartesian_spline.hpp"
#include "motion/core.hpp"
#include "motion/core/motion_types.hpp"
#include "motion/differential_math.hpp"
#include "motion/envelope_constraints.hpp"
#include "motion/interval_kinematics.hpp"
#include "motion/inverse_differential.hpp"
#include "motion/io/robot_model_import.hpp"
#include "motion/jerk_phase.hpp"
#include "motion/joint_program.hpp"
#include "motion/kinematics.hpp"
#include "motion/kinematics/differential_kinematics.hpp"
#include "motion/kinematics/interval_kinematics.hpp"
#include "motion/kinematics/inverse_differential.hpp"
#include "motion/kinematics/reference_kinematics.hpp"
#include "motion/kinematics/serial_chain_kinematics.hpp"
#include "motion/math/derivative_jets.hpp"
#include "motion/model_import.hpp"
#include "motion/path.hpp"
#include "motion/path_timing.hpp"
#include "motion/planning/boundary_quintic.hpp"
#include "motion/planning/cartesian_path.hpp"
#include "motion/planning/cartesian_spline.hpp"
#include "motion/planning/constant_jerk_phase.hpp"
#include "motion/planning/envelope_constraints.hpp"
#include "motion/planning/path_time_parameterization.hpp"
#include "motion/planning/pose_path.hpp"
#include "motion/pose_path.hpp"
#include "motion/program/joint_program.hpp"
#include "motion/program/robot_program.hpp"
#include "motion/reference_kinematics.hpp"
#include "motion/robot_program.hpp"
#include "motion/serial.hpp"
#ifdef MOTION_WITH_RUCKIG
#include "motion/joint_stop.hpp"
#include "motion/planning/joint_stop.hpp"
#endif
#include "motion/online.hpp"
#include "motion/planning/joint_trajectory.hpp"
#include "motion/planning/online_joint_generator.hpp"
#include "motion/trajectory.hpp"
#ifdef MOTION_WITH_RUCKIG
#include "motion/online_joint.hpp"
#include "motion/planning/ruckig_joint_generator.hpp"
#endif
#include "motion/execution.hpp"
#include "motion/execution/boundary_polynomial_schedule.hpp"
#include "motion/execution/cycle_aligned_program.hpp"
#include "motion/execution/deadline_motion_executor.hpp"
#include "motion/execution/execution_schedule.hpp"
#include "motion/execution/live_motion_executor.hpp"
#include "motion/execution/setpoint_buffer.hpp"
#include "motion/execution_grid.hpp"
#include "motion/execution_schedule.hpp"
#include "motion/live_execution.hpp"
#include "motion/polynomial_schedule.hpp"
#include "motion/setpoint_buffer.hpp"
#ifdef MOTION_WITH_RUCKIG
#include "motion/execution/stop_schedules.hpp"
#include "motion/stop_execution.hpp"
#endif
#include "motion/conveyor_tracking.hpp"
#include "motion/tracking.hpp"
#include "motion/tracking/constant_velocity_predictor.hpp"
#include "motion/tracking/conveyor_target_predictor.hpp"
#include "motion/tracking/frame_snapshot.hpp"
#include "motion/tracking_frames.hpp"
#ifdef MOTION_WITH_RUCKIG
#include "motion/conveyor_following.hpp"
#include "motion/tracking/conveyor_following.hpp"
#endif
#include "motion/core/clock_time.hpp"
#include "motion/execution/execution_types.hpp"
#include "motion/linux_platform.hpp"
#include "motion/platform/linux_realtime.hpp"
#include <cassert>
#include <type_traits>
int main() {
  static_assert(std::is_same_v<motion::TrackingTime, motion::ClockTime>);
  const motion::ClockTime observation{1, 7, 0};
  assert(observation.seconds == 1 && observation.clock == 7);
  motion::CartesianGantry kinematics;
  assert(kinematics.ik({{0, 0, 0}, {1, 0, 0, 0}}, {0, 0, 0}));
}
