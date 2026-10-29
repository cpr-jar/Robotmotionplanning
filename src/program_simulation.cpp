#include "motion/execution.hpp"
#include "motion/robot_program.hpp"
#include <iomanip>
#include <locale>
#include <ostream>
namespace motion {
namespace {
template <std::size_t N>
ProgramSimulationReport simulate(const PreparedRobotProgram &prepared, std::ostream *csv) {
  const auto &program = *prepared.reference;
  const auto &grid = *prepared.schedule;
  BufferedMotionProducer<N> producer(prepared.schedule, grid.cycle_s());
  DeadlineMotionExecutor<N> executor(producer.initial_command(), prepared.schedule, 7,
                                     {0, grid.cycle_s() / 4, 0, 0});
  SpscSetpointBuffer<N, 32> buffer;
  FixedJointCommand<N> command;
  ProgramSimulationReport result;
  result.duration_s = grid.duration();
  result.peak_joint_velocity = Vector(N);
  result.peak_joint_acceleration = Vector(N);
  result.peak_joint_jerk = Vector(N);
  if (csv) {
    *csv << "tick,time_s,timestamp_s,segment,source_step";
    for (const char *prefix : {"q", "dq", "ddq", "jerk"})
      for (std::size_t i = 0; i < N; ++i)
        *csv << ',' << prefix << i;
    *csv << ",tcp_x,tcp_y,tcp_z,qw,qx,qy,qz";
    for (const char *prefix : {"tcp_v", "tcp_a", "tcp_j"})
      for (int i = 0; i < 6; ++i)
        *csv << ',' << prefix << i;
    *csv << '\n' << std::setprecision(17);
  }
  std::size_t segment = 0, boundary = 0;
  for (std::uint64_t tick = 0; tick <= grid.total_ticks(); ++tick) {
    if (producer.fill(buffer, 32).status == BufferFillStatus::Error)
      throw std::runtime_error("Simulation producer error");
    const double due = grid.initial_state().timestamp_s + grid.tick_time(tick);
    const auto status = executor.poll(buffer, {due, 7, 0}, command);
    if (status != (tick == grid.total_ticks() ? ExecutionStatus::Finished : ExecutionStatus::Running))
      throw std::runtime_error("Simulation dispatcher rejected command");
    if (boundary < grid.boundary_ticks().size() && grid.boundary_ticks()[boundary] == tick) {
      if (!buffer_detail::rest(command))
        throw std::runtime_error("Boundary not rest");
      ++boundary;
      ++result.rest_boundaries;
    }
    while (segment + 1 < grid.segment_count() && tick >= grid.boundary_ticks()[segment])
      ++segment;
    const double begin = segment ? grid.segment_end(segment - 1) : 0, span = grid.segment_duration(segment),
                 local = std::clamp(grid.tick_time(tick) - begin, 0., span),
                 original = local == span ? program.segment_duration(segment)
                                          : (local / span) * program.segment_duration(segment);
    auto point = program.sample_motion(segment, original);
    const double scale = grid.time_scale(segment);
    for (double &j : point.joint_jerk)
      j *= scale * scale * scale;
    point.joints = {Vector(command.q.begin(), command.q.end()), Vector(command.dq.begin(), command.dq.end()),
                    Vector(command.ddq.begin(), command.ddq.end()), command.timestamp_s};
    point.tcp = program.robot().kinematics->fk(point.joints.q);
    point.tcp_derivatives = program.robot().kinematics->differentiate(point.joints.q, point.joints.dq,
                                                                      point.joints.ddq, point.joint_jerk);
    const std::array<Twist, 3> d{point.tcp_derivatives.velocity, point.tcp_derivatives.acceleration,
                                 point.tcp_derivatives.jerk};
    for (std::size_t i = 0; i < N; ++i) {
      result.peak_joint_velocity[i] = std::max(result.peak_joint_velocity[i], std::abs(command.dq[i]));
      result.peak_joint_acceleration[i] =
          std::max(result.peak_joint_acceleration[i], std::abs(command.ddq[i]));
      result.peak_joint_jerk[i] = std::max(result.peak_joint_jerk[i], std::abs(point.joint_jerk[i]));
    }
    for (int o = 0; o < 3; ++o) {
      result.peak_tcp_linear[o] =
          std::max(result.peak_tcp_linear[o], std::hypot(d[o].data[0], d[o].data[1], d[o].data[2]));
      result.peak_tcp_angular[o] =
          std::max(result.peak_tcp_angular[o], std::hypot(d[o].data[3], d[o].data[4], d[o].data[5]));
    }
    if (csv) {
      *csv << tick << ',' << command.program_time_s << ',' << command.timestamp_s << ',' << segment + 1 << ','
           << program.instruction(segment).location.step;
      for (const auto *v : {&point.joints.q, &point.joints.dq, &point.joints.ddq, &point.joint_jerk})
        for (double x : *v)
          *csv << ',' << x;
      for (double x : point.tcp.position)
        *csv << ',' << x;
      for (double x : point.tcp.orientation)
        *csv << ',' << x;
      for (auto value : d)
        for (double x : value.data)
          *csv << ',' << x;
      *csv << '\n';
      if (!*csv)
        throw std::runtime_error("Cannot write CSV");
    }
    ++result.commands;
    result.final_state = std::move(point.joints);
  }
  if (!is_rest(result.final_state, 0) || result.rest_boundaries != grid.segment_count())
    throw std::runtime_error("Missing terminal rest");
  return result;
}
template <std::size_t N = 1>
ProgramSimulationReport select(std::size_t n, const PreparedRobotProgram &p, std::ostream *csv) {
  if (n == N)
    return simulate<N>(p, csv);
  if constexpr (N < 12)
    return select<N + 1>(n, p, csv);
  throw std::invalid_argument("Simulation supports 1..12 axes");
}
} // namespace
ProgramSimulationReport simulate_robot_program(const PreparedRobotProgram &p, std::ostream *csv,
                                               ProgramSimulationOptions options) {
  if (!p.reference || !p.schedule || !options.max_ticks || p.schedule->total_ticks() > options.max_ticks)
    throw std::invalid_argument("Invalid program or simulation tick budget exceeded");
  struct Format {
    std::ostream *s;
    std::ios::fmtflags flags{};
    std::streamsize precision{}, width{};
    std::locale locale;
    explicit Format(std::ostream *out) : s(out) {
      if (s) {
        flags = s->flags();
        precision = s->precision();
        width = s->width();
        locale = s->getloc();
        s->imbue(std::locale::classic());
        s->flags(std::ios::dec);
        s->width(0);
      }
    }
    ~Format() {
      if (s) {
        s->flags(flags);
        s->precision(precision);
        s->width(width);
        try {
          s->imbue(locale);
        } catch (...) {
        }
      }
    }
  } format(csv);
  return select(p.reference->robot().kinematics->dof(), p, csv);
}
} // namespace motion
