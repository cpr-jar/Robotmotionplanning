#include "motion/execution/boundary_polynomial_schedule.hpp"
#include "motion/execution/live_motion_executor.hpp"
#include "motion/tracking/conveyor_target_predictor.hpp"
#include <cassert>
#include <iostream>
#include <random>
using namespace motion;
namespace {
class ThrowingSchedule final : public IExecutionSchedule {
public:
  explicit ThrowingSchedule(std::shared_ptr<const IExecutionSchedule> valid) : valid_(std::move(valid)) {}
  double duration() const noexcept override { return valid_->duration(); }
  double cycle_s() const noexcept override { return valid_->cycle_s(); }
  std::uint64_t total_ticks() const noexcept override { return valid_->total_ticks(); }
  double tick_time(std::uint64_t tick) const noexcept override { return valid_->tick_time(tick); }
  const std::vector<std::uint64_t> &boundary_ticks() const noexcept override {
    return valid_->boundary_ticks();
  }
  const JointState &initial_state() const noexcept override { return valid_->initial_state(); }
  const JointLimits &joint_limits() const noexcept override { return valid_->joint_limits(); }
  JointState sample(double t) const override { return valid_->sample(t); }
  JointState sample_tick(std::uint64_t tick) const override {
    ++calls;
    if (tick == 1)
      throw 77; // Preserve even non-std provider exceptions for non-RT diagnostics.
    return valid_->sample_tick(tick);
  }
  mutable std::size_t calls{};

private:
  std::shared_ptr<const IExecutionSchedule> valid_;
};
} // namespace
int main() {
  const JointLimits limits{{-2}, {2}, {2}, {5}, {50}};
  auto p = plan_boundary_quintic(limits, {{0}, {0}, {0}, 10}, {{1}, {0}, {0}}, 2);
  assert(p);
  auto schedule = std::make_shared<BoundaryPolynomialSchedule>(p.trajectory, .01);
  auto initial = prepare_live_plan<1>(schedule, 10, {1, 0, 0, 7});
  assert(initial);
  const auto candidate = [&](std::uint64_t start, std::uint64_t gen = 2) {
    auto c = initial.plan->command(start);
    auto poly =
        plan_boundary_quintic(limits, {{c.q[0]}, {c.dq[0]}, {c.ddq[0]}, c.timestamp_s}, {{.3}, {0}, {0}}, 2);
    assert(poly);
    return prepare_live_plan<1>(std::make_shared<BoundaryPolynomialSchedule>(poly.trajectory, .01), 10,
                                {gen, 1, start, 7})
        .plan;
  };
  auto replacement = candidate(20);
  {
    auto faulty = std::make_shared<ThrowingSchedule>(schedule);
    BufferedMotionProducer<1> producer(faulty, .01);
    SpscSetpointBuffer<1, 8> ring;
    const auto result = producer.fill(ring, 8);
    assert(result.status == BufferFillStatus::Error && result.count == 1);
    assert(result.diagnostic.code == BufferFillFault::ReferenceFailure && result.diagnostic.tick == 1 &&
           result.diagnostic.cause);
    bool original_cause = false;
    try {
      std::rethrow_exception(result.diagnostic.cause);
    } catch (int code) {
      original_cause = code == 77;
    }
    assert(original_cause);
    const auto calls = faulty->calls;
    FixedJointCommand<1> prefix;
    assert(ring.try_pop(prefix) && prefix.tick == 0 && !ring.try_pop(prefix));
    for (int retry = 0; retry < 3; ++retry) {
      const auto failed = producer.fill(ring, 8);
      assert(failed.status == BufferFillStatus::Error && failed.count == 0 && failed.diagnostic.tick == 1 &&
             failed.diagnostic.cause == result.diagnostic.cause);
    }
    assert(faulty->calls == calls && !ring.try_pop(prefix));
  }
  // Duplicate/stale payloads, corrupt values and regressing clocks cannot advance output.
  for (int kind = 0; kind < 6; ++kind) {
    BufferedMotionProducer<1> producer(schedule, .01);
    DeadlineMotionExecutor<1> exec(producer.initial_command(), schedule, 7, {0, .001, 0, 0});
    SpscSetpointBuffer<1, 2> ring;
    FixedJointCommand<1> out;
    assert(ring.try_push(producer.initial_command()));
    assert(exec.poll(ring, {10, 7, 0}, out) == ExecutionStatus::Running);
    const auto saved = out;
    auto bad = initial.plan->command(1);
    if (kind == 0)
      bad = producer.initial_command(); // duplicate tick
    if (kind == 1)
      bad.timestamp_s = 10; // stale timestamp
    if (kind == 2)
      bad.q[0] = NAN;
    if (kind == 3)
      bad.q[0] = 3;
    if (kind == 4)
      bad.terminal = true;
    assert(ring.try_push(bad));
    assert(exec.poll(ring, {kind == 5 ? 9.99 : 10.01, 7, 0}, out) == ExecutionStatus::Error);
    assert(exec.fault() == (kind == 5 ? ExecutionFault::ClockRegression : ExecutionFault::InvalidCommand));
    assert(out == saved);
    assert(exec.poll(ring, {10.01, 7, 0}, out) == ExecutionStatus::Error && out == saved);
  }
  for (int kind = 0; kind < 5; ++kind) {
    LivePlanMailbox<1> box;
    LiveMotionExecutor<1> exec(*initial.plan, box, {0, .001, .00001, 0});
    LiveJointCommand<1> out;
    assert(box.try_offer(*replacement));
    assert(!box.try_offer(*replacement));
    exec.service_requests();
    assert(!box.try_offer(*replacement));
    ClockTime now{10, 7, 0};
    if (kind == 0)
      now.clock = 8;
    if (kind == 1)
      now.uncertainty_s = .01;
    if (kind == 2)
      now.seconds = NAN;
    if (kind == 3)
      now.seconds = 10.002;
    if (kind == 4)
      now.uncertainty_s = -1;
    auto saved = out;
    assert(exec.poll(now, out) == ExecutionStatus::Error);
    assert(out.joints == saved.joints && out.generation == saved.generation);
    LivePlanResponse response;
    assert(box.try_response(response) && response.status == LiveResponseStatus::Faulted);
    assert(exec.poll({10, 7, 0}, out) == ExecutionStatus::Error);
  }
  {
    LivePlanMailbox<1> box;
    LiveMotionExecutor<1> exec(*initial.plan, box, {0, .001, 0, 0});
    LiveJointCommand<1> out;
    for (std::uint64_t tick = 0; tick <= 20; ++tick)
      assert(exec.poll({10 + schedule_detail::tick_time(tick, .01), 7, 0}, out) == ExecutionStatus::Running);
    assert(box.try_offer(*replacement));
    exec.service_requests();
    LivePlanResponse r;
    assert(box.try_response(r) && r.status == LiveResponseStatus::Late);
    assert(exec.generation() == 1);
  }
  {
    LivePlanMailbox<1> box;
    LiveMotionExecutor<1> exec(*initial.plan, box, {0, .001, 0, 0});
    assert(box.try_offer(*replacement));
    exec.service_requests();
    assert(exec.commit_delivery(1, 20));
    LivePlanResponse r;
    assert(box.try_response(r) && r.status == LiveResponseStatus::Committed);
    assert(!exec.commit_delivery(2, 21));
  }
  // Reproducible random refill/backpressure schedules: every tick exactly once.
  std::mt19937 random(0xFA017);
  for (int run = 0; run < 100; ++run) {
    BufferedMotionProducer<1> producer(schedule, .01);
    DeadlineMotionExecutor<1> exec(producer.initial_command(), schedule, 7, {0, .001, 0, 0});
    SpscSetpointBuffer<1, 2> ring;
    FixedJointCommand<1> out;
    for (std::uint64_t tick = 0; tick <= schedule->total_ticks(); ++tick) {
      for (unsigned i = 0; i < random() % 5; ++i)
        producer.fill(ring, random() % 8);
      producer.fill(ring, 2);
      assert(producer.fill(ring, 10).count == 0); // full ring must apply backpressure
      auto saved = out;
      assert(exec.poll(ring, {10 + schedule->tick_time(tick) - .002, 7, 0}, out) ==
             ExecutionStatus::Inactive);
      assert(out == saved);
      auto status = exec.poll(ring, {10 + schedule->tick_time(tick), 7, 0}, out);
      assert(status ==
             (tick == schedule->total_ticks() ? ExecutionStatus::Finished : ExecutionStatus::Running));
      assert(out.tick == tick);
    }
    auto saved = out;
    assert(exec.poll(ring, {100, 7, 0}, out) == ExecutionStatus::Inactive && out == saved);
  }
  {
    BufferedMotionProducer<1> producer(schedule, .01);
    DeadlineMotionExecutor<1> exec(producer.initial_command(), schedule, 7, {0, .001, 0, 0});
    SpscSetpointBuffer<1, 2> ring;
    FixedJointCommand<1> out;
    assert(exec.poll(ring, {10, 7, 0}, out) == ExecutionStatus::Error);
    assert(exec.fault() == ExecutionFault::Underflow);
    producer.fill(ring, 2);
    assert(exec.poll(ring, {10, 7, 0}, out) == ExecutionStatus::Error);
  }
  std::cout << "Fault injections and 100 randomized backpressure schedules: no duplicate, stale or silently "
               "resumed command\n";
}
