// Executable contract demonstration with an ideal clock and a simulated axis interface.
// This is not a production controller adapter and performs no hardware I/O.
#include "motion/execution/live_motion_executor.hpp"
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
  bool try_write(const AxisCommand &command) noexcept {
    if (stop_requests || command.tick != accepted_commands ||
        (reject_enabled && command.tick == rejected_tick))
      return false;
    last_command = command;
    ++accepted_commands;
    return true;
  }
  void request_stop() noexcept { ++stop_requests; }
  AxisCommand last_command{};
  std::uint64_t accepted_commands{}, stop_requests{}, rejected_tick{};
  bool reject_enabled{};
};

int run_contract(const std::string &filename, unsigned cycle_ms, std::string_view scenario) {
  using namespace motion;
  // Preparation phase: parsing, planning, resampling and allocation happen here.
  const auto prepared_program = load_robot_program(filename);
  if (prepared_program.reference->initial_state().q.size() != axis_count)
    throw std::invalid_argument("This demonstration requires a six-axis program");
  const double cycle_period_s = cycle_ms * .001;
  const auto aligned = align_program_cycles(prepared_program.reference, cycle_period_s);
  if (!aligned)
    throw std::invalid_argument(aligned.diagnostic);
  const double execution_epoch_s = aligned.program->initial_state().timestamp_s;
  const auto prepared_cache =
      prepare_live_plan<axis_count>(aligned.program, execution_epoch_s, {1, 0, 0, controller_clock_id});
  if (!prepared_cache)
    throw std::invalid_argument(prepared_cache.diagnostic);

  // The cache owner must outlive executor and every use of its raw plan pointer.
  LivePlanMailbox<axis_count> plan_mailbox;
  const ExecutionTimingPolicy timing_policy{0, cycle_period_s * .1, 0, 0};
  LiveMotionExecutor<axis_count> motion_executor(*prepared_cache.plan, plan_mailbox, timing_policy);
  SimulatedAxisInterface axes;
  VirtualControllerClock clock;
  const auto fault_tick = std::min<std::uint64_t>(5, prepared_cache.plan->end_tick());
  axes.reject_enabled = scenario == "sink-refusal";
  axes.rejected_tick = fault_tick;
  LiveJointCommand<axis_count> next_command;
  bool application_fault_latched = false;
  const auto latch_fault = [&]() noexcept {
    if (!application_fault_latched) {
      application_fault_latched = true;
      axes.request_stop(); // Real stop handling belongs to the host controller.
    }
    return ExecutionStatus::Error;
  };
  const auto dispatch_cycle = [&]() noexcept {
    if (application_fault_latched)
      return ExecutionStatus::Error;
    const auto execution_status = motion_executor.poll_clock(clock, next_command);
    if (execution_status == ExecutionStatus::Error)
      return latch_fault();
    if (execution_status == ExecutionStatus::Running || execution_status == ExecutionStatus::Finished) {
      // Finished carries the terminal command and must also be delivered exactly once.
      if (!axes.try_write(next_command.joints))
        return latch_fault();
    }
    return execution_status;
  };

  // Virtual cyclic phase: no parsing, planning, sample(), logging or allocations.
  clock.observation = {execution_epoch_s - cycle_period_s, controller_clock_id, 0};
  if (dispatch_cycle() != ExecutionStatus::Inactive || axes.accepted_commands != 0)
    return 1;
  bool observed_expected_fault = false;
  for (std::uint64_t tick = 0; tick <= prepared_cache.plan->end_tick(); ++tick) {
    clock.observation = {execution_epoch_s + aligned.program->tick_time(tick), controller_clock_id, 0};
    if (scenario == "late-clock" && tick == fault_tick)
      clock.observation.seconds += cycle_period_s * .5;
    const auto execution_status = dispatch_cycle();
    if (execution_status == ExecutionStatus::Error) {
      observed_expected_fault = scenario != "normal" && tick == fault_tick;
      if (scenario == "late-clock" && motion_executor.fault() != ExecutionFault::DeadlineMissed)
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
    if (execution_status !=
        (tick == prepared_cache.plan->end_tick() ? ExecutionStatus::Finished : ExecutionStatus::Running))
      return 1;
  }
  if (scenario == "normal") {
    const auto delivered_before_retry = axes.accepted_commands;
    clock.observation.seconds += cycle_period_s;
    if (dispatch_cycle() != ExecutionStatus::Inactive || axes.accepted_commands != delivered_before_retry ||
        axes.stop_requests || !axes.last_command.terminal || !buffer_detail::rest(axes.last_command) ||
        axes.accepted_commands != aligned.program->total_ticks() + 1)
      return 1;
  } else if (!observed_expected_fault)
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
                                  "[normal|late-clock|sink-refusal]");
    unsigned cycle_ms = 10;
    if (argc >= 3) {
      const std::string_view input = argv[2];
      const auto parsed = std::from_chars(input.data(), input.data() + input.size(), cycle_ms);
      if (parsed.ec != std::errc{} || parsed.ptr != input.data() + input.size() || cycle_ms < 1 ||
          cycle_ms > 20)
        throw std::invalid_argument("cycle_ms must be an integer from 1 to 20");
    }
    const std::string_view scenario = argc == 4 ? argv[3] : "normal";
    if (scenario != "normal" && scenario != "late-clock" && scenario != "sink-refusal")
      throw std::invalid_argument("Unknown contract scenario");
    return run_contract(argv[1], cycle_ms, scenario);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
