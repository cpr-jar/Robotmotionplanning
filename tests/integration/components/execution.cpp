#include "motion/execution/live_motion_session.hpp"
#include <cassert>
#ifdef MOTION_WITH_SERIAL
#error "Execution-only consumer must not enable the serial solver"
#endif
#ifdef MOTION_WITH_RUCKIG
#error "Execution-only consumer must not enable Ruckig"
#endif
namespace {
struct Schedule final : motion::IExecutionSchedule {
  motion::JointState initial{{0}, {0}, {0}, 0};
  motion::JointLimits limits{{-1}, {1}, {1}, {1}, {1}};
  std::vector<std::uint64_t> rest{2};
  double duration() const noexcept override { return .02; }
  double cycle_s() const noexcept override { return .01; }
  std::uint64_t total_ticks() const noexcept override { return 2; }
  double tick_time(std::uint64_t tick) const noexcept override {
    return motion::schedule_detail::tick_time(tick, .01);
  }
  const std::vector<std::uint64_t> &boundary_ticks() const noexcept override { return rest; }
  const motion::JointState &initial_state() const noexcept override { return initial; }
  const motion::JointLimits &joint_limits() const noexcept override { return limits; }
  motion::JointState sample(double t) const override {
    auto state = initial;
    state.timestamp_s = t;
    return state;
  }
  motion::JointState sample_tick(std::uint64_t tick) const override { return sample(tick_time(tick)); }
};
struct Sink {
  std::uint64_t count{};
  bool try_write(const motion::LiveJointCommand<1> &command) noexcept {
    assert(command.joints.tick == count++);
    return true;
  }
  void request_stop() noexcept { assert(false); }
};
} // namespace
int main() {
  auto plan = motion::prepare_live_plan<1>(std::make_shared<Schedule>(), 0, {1, 0, 0, 7});
  assert(plan);
  motion::LiveMotionSession<1> session(std::move(plan.plan), {0, .001, 0, 0});
  Sink sink;
  for (std::uint64_t tick = 0; tick <= 2; ++tick)
    assert(session.dispatch({motion::schedule_detail::tick_time(tick, .01), 7, 0}, sink) ==
           (tick == 2 ? motion::ExecutionStatus::Finished : motion::ExecutionStatus::Running));
  session.plans().request_shutdown();
  assert(session.dispatch({.03, 7, 0}, sink) == motion::ExecutionStatus::Inactive);
  assert(session.plans().release_after_shutdown() && sink.count == 3);
}
