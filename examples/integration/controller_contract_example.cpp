// Executable contract demonstration with an ideal clock and a simulated axis interface.
// This is not a production controller adapter and performs no hardware I/O.
#include "motion/execution/live_motion_session.hpp"
#include "motion/program/robot_program.hpp"
#include <charconv>
#include <iostream>
#include <string_view>

namespace {
constexpr std::size_t axis_count = 6;
constexpr motion::ClockId controller_clock_id = 1;
using AxisCommand = motion::FixedJointCommand<axis_count>;

class VirtualControllerClock {
public:
  motion::ClockTime now() const noexcept { return observation; }
  motion::ClockTime observation;
};

// Example boundary owned by the application, deliberately outside the public library API.
class SimulatedAxisInterface {
public:
  bool try_write(const motion::LiveJointCommand<axis_count> &command) noexcept {
    if (stop_requests || command.joints.tick != accepted_commands ||
        (reject_enabled && command.joints.tick == rejected_tick))
      return false;
    last_command = command.joints;
    last_generation = command.generation;
    ++accepted_commands;
    return true;
  }
  void request_stop() noexcept { ++stop_requests; }
  AxisCommand last_command{};
  std::uint64_t accepted_commands{}, stop_requests{}, rejected_tick{}, last_generation{};
  bool reject_enabled{};
};

int run_contract(const std::string &filename, unsigned cycle_ms, std::string_view scenario) {
  using namespace motion;
  // Preparation phase: parsing, planning, resampling and allocation happen here.
  const auto prepared_program = load_robot_program(filename);
  if (prepared_program.reference->initial_state().q.size() != axis_count)
    throw std::invalid_argument("This demonstration requires a six-axis program");
  const double cycle_period_s = cycle_ms * .001;
  const auto aligned = retime_robot_program(prepared_program, cycle_period_s);
  const double execution_epoch_s = aligned.schedule->initial_state().timestamp_s;
  auto prepared_cache =
      prepare_live_plan<axis_count>(aligned.schedule, execution_epoch_s, {1, 0, 0, controller_clock_id});
  if (!prepared_cache)
    throw std::invalid_argument(prepared_cache.diagnostic);

  const auto terminal_tick = prepared_cache.plan->end_tick();
  // Session owns caches; management releases them only after cyclic quiescence.
  const ExecutionTimingPolicy timing_policy{0, cycle_period_s * .1, 0, 0};
  LiveMotionSession<axis_count> motion_session(std::move(prepared_cache.plan), timing_policy);
  SimulatedAxisInterface axes;
  VirtualControllerClock clock;
  const auto fault_tick = std::min<std::uint64_t>(5, terminal_tick);
  axes.reject_enabled = scenario == "sink-refusal";
  axes.rejected_tick = fault_tick;
  const auto dispatch_cycle = [&]() noexcept { return motion_session.dispatch_clock(clock, axes); };

  // Virtual cyclic phase: no parsing, planning, sample(), logging or allocations.
  clock.observation = {execution_epoch_s - cycle_period_s, controller_clock_id, 0};
  if (dispatch_cycle() != ExecutionStatus::Inactive || axes.accepted_commands != 0)
    return 1;
  bool observed_expected_termination = false;
  for (std::uint64_t tick = 0; tick <= terminal_tick; ++tick) {
    clock.observation = {execution_epoch_s + aligned.schedule->tick_time(tick), controller_clock_id, 0};
    if (scenario == "late-clock" && tick == fault_tick)
      clock.observation.seconds += cycle_period_s * .5;
    if (scenario == "shutdown" && tick == fault_tick)
      motion_session.plans().request_shutdown(); // Atomic request; no cache deletion here.
    const auto execution_status = dispatch_cycle();
    if (scenario == "shutdown" && tick == fault_tick) {
      if (execution_status != ExecutionStatus::Inactive || motion_session.diagnostic() ||
          axes.accepted_commands != fault_tick || axes.stop_requests != 1)
        return 1;
      observed_expected_termination = true;
      break;
    }
    if (execution_status == ExecutionStatus::Error) {
      observed_expected_termination = scenario != "normal" && tick == fault_tick;
      const auto diagnostic = motion_session.diagnostic();
      if (!diagnostic || diagnostic->tick != fault_tick ||
          (scenario == "late-clock" && diagnostic->execution_fault != ExecutionFault::DeadlineMissed) ||
          (scenario == "sink-refusal" && diagnostic->code != LiveSessionFaultCode::SinkRejected))
        return 1;
      const auto delivered_before_retry = axes.accepted_commands;
      // Fault remains latched even if a later clock reading is valid.
      for (int attempt = 0; attempt < 3; ++attempt) {
        clock.observation.seconds += cycle_period_s;
        if (dispatch_cycle() != ExecutionStatus::Error)
          return 1;
      }
      if (axes.accepted_commands != delivered_before_retry || axes.stop_requests != 1 ||
          axes.accepted_commands != fault_tick)
        return 1;
      break;
    }
    if (execution_status != (tick == terminal_tick ? ExecutionStatus::Finished : ExecutionStatus::Running))
      return 1;
  }
  if (scenario == "normal") {
    const auto delivered_before_retry = axes.accepted_commands;
    clock.observation.seconds += cycle_period_s;
    if (dispatch_cycle() != ExecutionStatus::Inactive || axes.accepted_commands != delivered_before_retry ||
        axes.stop_requests || !axes.last_command.terminal || !buffer_detail::rest(axes.last_command) ||
        axes.accepted_commands != aligned.schedule->total_ticks() + 1 || axes.last_generation != 1)
      return 1;
  } else if (!observed_expected_termination)
    return 1;

  // Virtual RT shutdown acknowledgement. A real host keeps cycling until this
  // acknowledgement, then joins its cyclic thread before destroying the session.
  motion_session.plans().request_shutdown();
  clock.observation.seconds += cycle_period_s;
  const auto shutdown_status = dispatch_cycle();
  if (shutdown_status != (motion_session.diagnostic() ? ExecutionStatus::Error : ExecutionStatus::Inactive))
    return 1;
  // Management phase: dropping shared_ptrs and freeing caches is allowed here.
  if (!motion_session.plans().release_after_shutdown() || motion_session.plans().retained_plan_count())
    return 1;

  // Diagnostics after the cyclic phase; ordinary console output is intentionally outside it.
  std::cout << "CONTRACT PASS: scenario=" << scenario << ", cycle_ms=" << cycle_ms
            << ", accepted_commands=" << axes.accepted_commands << ", stop_requests=" << axes.stop_requests
            << "; ideal virtual clock, simulated axes\n";
  return 0;
}
} // namespace

int main(int argc, char **argv) {
  try {
    if (argc < 2 || argc > 4)
      throw std::invalid_argument("Usage: controller_contract_example PROGRAM.yaml [cycle_ms:1..20] "
                                  "[normal|late-clock|sink-refusal|shutdown]");
    unsigned cycle_ms = 10;
    if (argc >= 3) {
      const std::string_view input = argv[2];
      const auto parsed = std::from_chars(input.data(), input.data() + input.size(), cycle_ms);
      if (parsed.ec != std::errc{} || parsed.ptr != input.data() + input.size() || cycle_ms < 1 ||
          cycle_ms > 20)
        throw std::invalid_argument("cycle_ms must be an integer from 1 to 20");
    }
    const std::string_view scenario = argc == 4 ? argv[3] : "normal";
    if (scenario != "normal" && scenario != "late-clock" && scenario != "sink-refusal" &&
        scenario != "shutdown")
      throw std::invalid_argument("Unknown contract scenario");
    return run_contract(argv[1], cycle_ms, scenario);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
