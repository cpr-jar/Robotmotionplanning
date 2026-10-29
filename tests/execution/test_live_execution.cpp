#include "motion/execution/boundary_polynomial_schedule.hpp"
#include "motion/execution/live_motion_executor.hpp"
#include <cassert>
#include <iostream>
#include <thread>
using namespace motion;
namespace {
const JointLimits limits{{-2}, {2}, {2}, {5}, {50}};
std::shared_ptr<const BoundaryPolynomialSchedule> schedule(const JointState &s, double q,
                                                           double duration = 2) {
  auto p = plan_boundary_quintic(limits, s, {{q}, {0}, {0}}, duration);
  assert(p);
  return std::make_shared<BoundaryPolynomialSchedule>(p.trajectory, .01);
}
} // namespace
int main() {
  const auto original = schedule({{0}, {0}, {0}, 10}, 1);
  const auto initial = prepare_live_plan<1>(original, 10, {1, 0, 0, 7});
  assert(initial);
  const auto boundary = initial.plan->command(50);
  const JointState at{{boundary.q[0]}, {boundary.dq[0]}, {boundary.ddq[0]}, boundary.timestamp_s};
  const auto replacement = prepare_live_plan<1>(schedule(at, .3), 10, {2, 1, 50, 7});
  assert(replacement);
  LivePlanMailbox<1> mailbox;
  LiveMotionExecutor<1> executor(*initial.plan, mailbox, {0, .001, 0, 0});
  assert(mailbox.try_offer(*replacement.plan));
  LiveJointCommand<1> out;
  std::atomic<bool> done{};
  std::thread observer([&] {
    while (!done.load()) {
      if (auto s = executor.snapshot()) {
        assert(s->generation == 1 || s->generation == 2);
        if (s->generation == 2)
          assert(s->next_tick >= 51);
      }
    }
  });
  std::size_t activated = 0;
  for (std::uint64_t tick = 0; tick <= replacement.plan->end_tick(); ++tick) {
    const auto status = executor.poll({10 + schedule_detail::tick_time(tick, .01), 7, 0}, out);
    assert(status ==
           (tick == replacement.plan->end_tick() ? ExecutionStatus::Finished : ExecutionStatus::Running));
    assert(out.generation == (tick < 50 ? 1 : 2));
    assert(out.joints.tick == tick);
    assert(executor.commit_delivery(out.generation, tick));
    LivePlanResponse response;
    if (mailbox.try_response(response)) {
      assert(tick == 50 && response.status == LiveResponseStatus::Activated &&
             response.retired_generation == 1);
      ++activated;
    }
  }
  done = true;
  observer.join();
  assert(activated == 1 && buffer_detail::rest(out.joints));
  {
    LivePlanMailbox<1> box;
    LiveMotionExecutor<1> execution(*initial.plan, box, {0, .001, 0, 0});
    assert(execution.commit_delivery(1, 50));
    assert(box.try_offer(*replacement.plan));
    execution.service_requests();
    LivePlanResponse r;
    assert(box.try_response(r) && r.status == LiveResponseStatus::Committed);
    assert(execution.generation() == 1);
  }
  {
    LivePlanMailbox<1> box;
    LiveMotionExecutor<1> execution(*initial.plan, box, {0, .001, 0, 0});
    const auto bad = prepare_live_plan<1>(schedule(at, .3), 10, {2, 999, 50, 7});
    assert(!bad);
    const auto stale = prepare_live_plan<1>(schedule(at, .3), 10, {1000, 999, 50, 7});
    assert(stale);
    std::atomic<bool> complete{};
    std::thread planner([&] {
      for (int i = 0; i < 10000; ++i) {
        while (!box.try_offer(*stale.plan)) {
        }
        LivePlanResponse response;
        while (!box.try_response(response)) {
        }
        assert(response.status == LiveResponseStatus::IdentityMismatch);
      }
      complete = true;
    });
    while (!complete.load())
      execution.service_requests();
    planner.join();
  }
  {
    BufferedMotionProducer<1> producer(original, .01);
    DeadlineMotionExecutor<1> execution(producer.initial_command(), original, 7, {0, .001, 0, 0});
    SpscSetpointBuffer<1, 8> ring;
    FixedJointCommand<1> c;
    producer.fill(ring, 8);
    const auto saved = c;
    assert(execution.poll(ring, {10.01, 7, 0}, c) == ExecutionStatus::Error);
    assert(execution.fault() == ExecutionFault::DeadlineMissed && c == saved);
    assert(execution.poll(ring, {10, 7, 0}, c) == ExecutionStatus::Error);
  }
  {
    BufferedMotionProducer<1> producer(original, .01);
    DeadlineMotionExecutor<1> execution(producer.initial_command(), original, 7, {0, .001, 0, 0});
    SpscSetpointBuffer<1, 8> ring;
    FixedJointCommand<1> c;
    producer.fill(ring, 8);
    struct Clock {
      int reads{};
      ClockTime now() { return {reads++ ? 10.002 : 10., 7, 0}; }
    } clock;
    assert(execution.poll_clock(ring, clock, c) == ExecutionStatus::Error &&
           execution.fault() == ExecutionFault::DeadlineMissed);
  }
  {
    SpscSetpointBuffer<1, 16> ring;
    std::thread writer([&] {
      for (std::uint64_t i = 0; i < 100000; ++i) {
        FixedJointCommand<1> c;
        c.tick = i;
        while (!ring.try_push(c)) {
        }
      }
    });
    for (std::uint64_t i = 0; i < 100000; ++i) {
      FixedJointCommand<1> c;
      while (!ring.try_pop(c)) {
      }
      assert(c.tick == i);
    }
    writer.join();
  }
  std::cout << "Live exact-state switch, coherent snapshots, driver fence, clock/preemption faults and "
               "two-thread mailbox/ring stress passed\n";
}
