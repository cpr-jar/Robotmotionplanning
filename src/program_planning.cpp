#include "motion/envelope_constraints.hpp"
#include "motion/robot_program.hpp"
namespace motion {
namespace {
class HoldTrajectory final : public ITrajectory {
public:
  HoldTrajectory(JointState s, double t) : state_(std::move(s)), duration_(t) {}
  double duration() const noexcept override { return duration_; }
  JointState sample(double t) const override {
    if (!std::isfinite(t) || t < 0 || t > duration_)
      throw std::invalid_argument("WAIT time outside interval");
    auto s = state_;
    s.timestamp_s += t;
    return s;
  }

private:
  JointState state_;
  double duration_;
};
JointLimits scaled(JointLimits l, double s) {
  for (std::size_t i = 0; i < l.max_velocity.size(); ++i) {
    l.max_velocity[i] *= s;
    l.max_acceleration[i] *= s * s;
    l.max_jerk[i] *= s * s * s;
  }
  return l;
}
CartesianLimits scaled(CartesianLimits l, double s) {
  l.max_linear_velocity *= s;
  l.max_angular_velocity *= s;
  l.max_linear_acceleration *= s * s;
  l.max_angular_acceleration *= s * s;
  l.max_linear_jerk *= s * s * s;
  l.max_angular_jerk *= s * s * s;
  return l;
}
} // namespace
JointState RobotProgram::sample_segment(std::size_t i, double t) const {
  if (!std::isfinite(t) || t < 0 || t > segment_duration(i))
    throw std::invalid_argument("Segment sample outside interval");
  auto r = segments_.at(i)->sample(t);
  r.timestamp_s = initial_.timestamp_s + (i ? ends_[i - 1] : 0) + t;
  return r;
}
JointState RobotProgram::sample(double t) const {
  if (!std::isfinite(t) || t < 0 || t > duration())
    throw std::invalid_argument("Program sample outside interval");
  const auto it = std::upper_bound(ends_.begin(), ends_.end(), t);
  const auto i = it == ends_.end() ? ends_.size() - 1 : std::size_t(it - ends_.begin());
  return sample_segment(i, t == ends_[i] ? segment_duration(i) : t - (i ? ends_[i - 1] : 0));
}
TimedPathSample RobotProgram::sample_motion(std::size_t i, double t) const {
  TimedPathSample out;
  out.joints = sample_segment(i, t);
  out.joint_jerk = Vector(out.joints.q.size());
  if (const auto *p = dynamic_cast<const BoundaryQuinticTrajectory *>(segments_.at(i).get()))
    out.joint_jerk = p->sample_with_jerk(t).second;
  else if (const auto *p = dynamic_cast<const TimedPathTrajectory *>(segments_.at(i).get()))
    out.joint_jerk = p->sample_motion(t).joint_jerk;
  out.tcp = robot_.kinematics->fk(out.joints.q);
  out.tcp_derivatives =
      robot_.kinematics->differentiate(out.joints.q, out.joints.dq, out.joints.ddq, out.joint_jerk);
  return out;
}
PreparedRobotProgram compile_robot_program(const RobotProgramSource &source, ImportedSerialRobot robot) {
  const ProgramLocation root{source.filename(), "program", 1, 1, 0};
  if (!robot.kinematics)
    throw RobotProgramError(root, "Missing robot kinematics");
  try {
    const auto n = robot.kinematics->dof();
    check_limits(robot.joint_limits, n);
    if (!kinematic_detail::finite(source.start(), n))
      throw RobotProgramError(source.start_location(), "Start joint dimension does not match robot");
    const JointState initial{source.start(), Vector(n), Vector(n), source.epoch_s()};
    if (!within_joint_limits(initial, robot.joint_limits, 0))
      throw RobotProgramError(source.start_location(), "Initial joints exceed profile limits");
    robot.kinematics->fk(initial.q);
    auto program = std::shared_ptr<RobotProgram>(new RobotProgram(std::move(robot)));
    program->initial_ = initial;
    auto state = initial;
    double total = 0;
    for (const auto &instruction : source.instructions()) {
      try {
        const auto limits = scaled(program->robot_.joint_limits, instruction.speed);
        const auto tcp = scaled(program->robot_.tcp_limits, instruction.speed);
        TrajectoryPtr segment;
        const auto target_pose = [&]() {
          if (const auto *p = std::get_if<Pose>(&instruction.target))
            return *p;
          const auto &q = std::get<Vector>(instruction.target);
          if (!kinematic_detail::finite(q, n))
            throw std::invalid_argument("Target joint dimension does not match robot");
          if (!within_joint_limits({q, Vector(n), Vector(n), 0}, limits, 0))
            throw std::invalid_argument("Target joints exceed profile");
          return program->robot_.kinematics->fk(q);
        };
        if (instruction.operation == ProgramOperation::Wait)
          segment = std::make_shared<HoldTrajectory>(state, instruction.seconds);
        else if (instruction.operation == ProgramOperation::Ptp) {
          Vector destination;
          if (const auto *q = std::get_if<Vector>(&instruction.target))
            destination = *q;
          else {
            const auto solution = program->robot_.kinematics->ik(target_pose(), state.q);
            if (!solution)
              throw std::domain_error("PTP target has no seed-local IK solution");
            destination = *solution;
          }
          if (!kinematic_detail::finite(destination, n))
            throw std::invalid_argument("Target joint dimension does not match robot");
          const JointTarget goal{destination, Vector(n), Vector(n)};
          const auto joint = QuinticJointPlanner(limits).plan(state, goal);
          if (!joint)
            throw std::domain_error(joint.diagnostic);
          double T = joint.trajectory->duration();
          for (int attempt = 0; attempt < 16 && T <= 3600; ++attempt, T *= 1.5) {
            auto p = plan_boundary_quintic(limits, state, goal, T);
            if (p && check_polynomial_constraints(*program->robot_.kinematics, *p.trajectory, tcp)) {
              segment = p.trajectory;
              break;
            }
          }
          if (!segment)
            throw std::domain_error("PTP TCP/joint envelope rejected within duration budget");
        } else {
          const auto first = program->robot_.kinematics->fk(state.q);
          std::shared_ptr<const IFixedOrientationPath> geometry;
          auto orientation = first.orientation;
          if (instruction.operation == ProgramOperation::Line) {
            auto goal = target_pose();
            orientation = goal.orientation;
            geometry = std::make_shared<CartesianLinePath>(first, Pose{goal.position, first.orientation});
          } else {
            geometry = std::make_shared<CartesianArcPath>(
                instruction.relative_center ? kinematic_detail::add(first.position, instruction.center)
                                            : instruction.center,
                first, instruction.normal, instruction.sweep_rad);
            if (instruction.orientation)
              orientation = *instruction.orientation;
          }
          const auto path = map_cartesian_spline(
              program->robot_.kinematics, std::make_shared<PoseSweepPath>(geometry, orientation), state.q);
          if (!path)
            throw std::domain_error(path.diagnostic);
          auto timed = PathTimeParameterizer(limits).plan(path.path, state, tcp);
          if (!timed)
            throw std::domain_error(timed.diagnostic);
          segment = timed.trajectory;
        }
        const double next = total + segment->duration();
        if (!std::isfinite(next) || next <= total || next > 86400 ||
            !std::isfinite(initial.timestamp_s + next) || initial.timestamp_s + next <= state.timestamp_s ||
            std::abs((next - total) - segment->duration()) > 1e-12 * segment->duration())
          throw std::domain_error("Program duration/clock budget exceeded");
        const auto first = segment->sample(0);
        auto last = segment->sample(segment->duration());
        if (!is_rest(first, 0) || !is_rest(last, 0))
          throw std::domain_error("Program endpoints must be rest");
        for (std::size_t i = 0; i < n; ++i)
          if (std::abs(first.q.at(i) - state.q[i]) > 1e-12)
            throw std::domain_error("Discontinuous program boundary");
        state = std::move(last);
        state.timestamp_s = initial.timestamp_s + next;
        program->segments_.push_back(std::move(segment));
        program->ends_.push_back(next);
        program->instructions_.push_back(instruction);
        total = next;
      } catch (const RobotProgramError &) {
        throw;
      } catch (const std::exception &e) {
        throw RobotProgramError(instruction.location, e.what());
      }
    }
    if (!program->segment_count())
      throw RobotProgramError(root, "Empty program");
    const auto grid = align_program_cycles(program, source.cycle_s());
    if (!grid)
      throw RobotProgramError(root, grid.diagnostic);
    const double magnitude = std::max(std::abs(initial.timestamp_s),
                                      std::abs(initial.timestamp_s + grid.program->duration())),
                 ulp = std::nextafter(magnitude, INFINITY) - magnitude;
    if (source.cycle_s() <= 64 * ulp)
      throw RobotProgramError(root, "Insufficient clock precision for dispatch");
    return {std::move(program), grid.program};
  } catch (const RobotProgramError &) {
    throw;
  } catch (const std::exception &e) {
    throw RobotProgramError(root, e.what());
  }
}
} // namespace motion
