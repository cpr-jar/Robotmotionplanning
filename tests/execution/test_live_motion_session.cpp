#include "motion/execution/boundary_polynomial_schedule.hpp"
#include "motion/execution/live_motion_session.hpp"
#include <cassert>
#include <iostream>
#include <thread>
#include <type_traits>
using namespace motion;
namespace {
using Plan = PreparedLivePlan<1>;
using PlanPtr = std::shared_ptr<const Plan>;
const JointLimits limits{{-2}, {2}, {2}, {5}, {50}};
constexpr ExecutionTimingPolicy timing{0, .001, 0, 0};
ClockTime at(std::uint64_t tick) {
  return {10 + schedule_detail::tick_time(tick, .01), 7, 0};
}
PlanPtr prepare(JointState start, double goal, LivePlanStamp stamp, double duration = 2) {
  auto polynomial = plan_boundary_quintic(limits, start, {{goal}, {0}, {0}}, duration);
  assert(polynomial);
  const auto result = prepare_live_plan<1>(
      std::make_shared<BoundaryPolynomialSchedule>(polynomial.trajectory, .01), 10, stamp);
  assert(result);
  return result.plan;
}
PlanPtr initial_plan() {
  return prepare({{0}, {0}, {0}, 10}, .2, {1, 0, 0, 7});
}
PlanPtr replacement(const Plan &source, std::uint64_t tick, std::uint64_t generation = 2) {
  const auto c = source.command(tick);
  return prepare({{c.q[0]}, {c.dq[0]}, {c.ddq[0]}, c.timestamp_s}, .1,
                 {generation, source.stamp().generation, tick, 7});
}
// The actual last release must run on the management thread, not merely expire
// a weak handle while some external reference keeps the cache alive.
PlanPtr management_owned(PlanPtr plan) {
  const auto *address = plan.get();
  return PlanPtr(address,
                 [keep = std::move(plan), thread = std::this_thread::get_id()](const Plan *) mutable {
                   assert(std::this_thread::get_id() == thread);
                   keep.reset();
                 });
}
struct Sink {
  std::uint64_t attempts{}, accepted{}, stops{}, refused_tick{UINT64_MAX};
  LiveJointCommand<1> last;
  bool try_write(const LiveJointCommand<1> &command) noexcept {
    ++attempts;
    if (stops || command.joints.tick != accepted || command.joints.tick == refused_tick)
      return false;
    last = command;
    ++accepted;
    return true;
  }
  void request_stop() noexcept { ++stops; }
};
void retirement_and_completion() {
  auto first = management_owned(initial_plan());
  auto next = management_owned(replacement(*first, 20));
  const auto end = next->end_tick();
  std::weak_ptr<const Plan> retired = first, active = next;
  LiveMotionSession<1> session(std::move(first), timing);
  Sink sink;
  assert(session.plans().try_offer(next));
  next.reset();
  assert(session.plans().retained_plan_count() == 2);
  for (std::uint64_t tick = 0; tick <= end; ++tick) {
    assert(session.dispatch(at(tick), sink) ==
           (tick == end ? ExecutionStatus::Finished : ExecutionStatus::Running));
    assert(sink.last.generation == (tick < 20 ? 1 : 2));
    const auto snapshot = session.snapshot();
    assert(snapshot && snapshot->next_tick == tick + 1 && snapshot->committed_through == tick);
    if (tick == 20) {
      assert(!retired.expired() && !active.expired());
      const auto response = session.plans().collect_response();
      assert(response && response->status == LiveResponseStatus::Activated &&
             response->retired_generation == 1);
      assert(retired.expired() && !active.expired());
    }
  }
  assert(session.dispatch(at(end + 1), sink) == ExecutionStatus::Inactive);
  assert(sink.accepted == end + 1 && sink.attempts == sink.accepted && sink.stops == 0);
  assert(!session.plans().release_after_shutdown());
  assert(session.plans().request_shutdown());
  assert(!session.plans().release_after_shutdown());
  assert(session.dispatch(at(end + 2), sink) == ExecutionStatus::Inactive);
  assert(session.plans().state() == LiveSessionState::Quiescent);
  assert(session.snapshot()->quiescent && session.snapshot()->generation == 0);
  assert(session.plans().release_after_shutdown() && active.expired());
  assert(session.plans().retained_plan_count() == 0);
  assert(!session.plans().request_shutdown() && !session.plans().request_cancel());
  assert(session.plans().release_after_shutdown());
  assert(session.dispatch(at(0), sink) == ExecutionStatus::Inactive); // No access to released caches.
  assert(!session.diagnostic() && sink.stops == 0 && !session.plans().collect_response());
}
void cancellation_and_backpressure() {
  auto first = management_owned(initial_plan());
  auto next = management_owned(replacement(*first, 20));
  auto same_generation = replacement(*first, 20);
  auto after_cancel = replacement(*first, 20, 3);
  std::weak_ptr<const Plan> cancelled = next;
  LiveMotionSession<1> session(std::move(first), timing);
  Sink sink;
  assert(session.plans().try_offer({}).status == LivePlanOfferStatus::InvalidPlan);
  assert(session.plans().try_offer(next));
  next.reset();
  assert(session.plans().try_offer(after_cancel).status == LivePlanOfferStatus::Busy);
  assert(session.plans().request_cancel());
  assert(session.dispatch(at(0), sink) == ExecutionStatus::Running);
  assert(!cancelled.expired());
  const auto response = session.plans().collect_response();
  assert(response && response->status == LiveResponseStatus::Cancelled && response->generation == 2);
  assert(cancelled.expired() && !session.plans().request_cancel());
  assert(session.plans().try_offer(same_generation).status == LivePlanOfferStatus::GenerationReused);
  assert(session.plans().last_offered_generation() == 2);
  assert(session.plans().try_offer(after_cancel));
  after_cancel.reset();
  for (std::uint64_t tick = 1; tick <= 20; ++tick)
    assert(session.dispatch(at(tick), sink) == ExecutionStatus::Running);
  // The response is already published: a cancellation request cannot undo activation.
  assert(session.plans().request_cancel());
  const auto activated = session.plans().collect_response();
  assert(activated && activated->status == LiveResponseStatus::Activated && activated->generation == 3);
  assert(session.dispatch(at(21), sink) == ExecutionStatus::Running);
  assert(sink.last.generation == 3 && !session.plans().collect_response());
  assert(session.plans().request_shutdown());
  assert(session.dispatch(at(22), sink) == ExecutionStatus::Inactive);
  assert(session.plans().release_after_shutdown() && sink.stops == 1);
}
void shutdown_outstanding() {
  // Before admission, after admission, and after activation with an unread reply.
  for (int stage = 0; stage < 3; ++stage) {
    auto first = management_owned(initial_plan());
    auto next = management_owned(replacement(*first, 20));
    auto extra = replacement(*first, 20, 3);
    std::weak_ptr<const Plan> old = first, fresh = next;
    LiveMotionSession<1> session(std::move(first), timing);
    Sink sink;
    assert(session.plans().try_offer(next));
    next.reset();
    if (stage > 0)
      for (std::uint64_t tick = 0; tick <= (stage == 2 ? 20u : 0u); ++tick)
        assert(session.dispatch(at(tick), sink) == ExecutionStatus::Running);
    assert(session.plans().request_shutdown());
    assert(session.plans().try_offer(extra).status == LivePlanOfferStatus::Stopping);
    assert(!session.plans().release_after_shutdown() && !old.expired() && !fresh.expired());
    const auto before = sink.accepted;
    assert(session.dispatch(at(21), sink) == ExecutionStatus::Inactive);
    assert(sink.accepted == before && sink.stops == 1);
    if (stage < 2) {
      const auto response = session.plans().collect_response();
      assert(response && response->status == LiveResponseStatus::Shutdown);
      assert(fresh.expired() && !old.expired());
    }
    // Quiescence also retires both caches if the activation reply was never collected.
    assert(session.plans().release_after_shutdown() && old.expired() && fresh.expired());
    assert(session.dispatch(at(0), sink) == ExecutionStatus::Inactive && sink.stops == 1);
  }
}
void faults_and_terminal_refusal() {
  for (int kind = 0; kind < 3; ++kind) {
    auto first = management_owned(initial_plan());
    auto next = management_owned(replacement(*first, 20));
    std::weak_ptr<const Plan> active = first, pending = next;
    LiveMotionSession<1> session(std::move(first), timing);
    Sink sink;
    assert(session.plans().try_offer(next));
    next.reset();
    const std::uint64_t failed_tick = kind == 2 ? 220 : 5;
    if (kind != 0)
      sink.refused_tick = failed_tick;
    for (std::uint64_t tick = 0; tick <= failed_tick; ++tick) {
      auto now = at(tick);
      if (kind == 0 && tick == failed_tick)
        now.seconds += .002;
      assert(session.dispatch(now, sink) ==
             (tick == failed_tick ? ExecutionStatus::Error : ExecutionStatus::Running));
    }
    const auto fault = session.diagnostic();
    assert(fault &&
           fault->code == (kind == 0 ? LiveSessionFaultCode::Execution : LiveSessionFaultCode::SinkRejected));
    assert(fault->tick == failed_tick && fault->generation == (kind == 2 ? 2u : 1u));
    assert(fault->execution_fault == (kind == 0 ? ExecutionFault::DeadlineMissed : ExecutionFault::None));
    const auto snapshot = session.snapshot();
    assert(snapshot && snapshot->quiescent && snapshot->committed_through == failed_tick - 1);
    assert(sink.accepted == failed_tick && sink.stops == 1);
    assert(session.plans().release_after_shutdown() && active.expired() && pending.expired());
    for (int retry = 0; retry < 3; ++retry)
      assert(session.dispatch(at(0), sink) == ExecutionStatus::Error);
    assert(sink.accepted == failed_tick && sink.stops == 1 && session.diagnostic()->tick == failed_tick);
  }
  {
    LiveMotionSession<1> session(initial_plan(), timing);
    Sink sink;
    struct Clock {
      int reads{};
      ClockTime now() noexcept { return {reads++ ? 10.002 : 10., 7, 0}; }
    } clock;
    assert(session.dispatch_clock(clock, sink) == ExecutionStatus::Error);
    assert(clock.reads == 2 && sink.attempts == 0 && sink.stops == 1);
    assert(session.diagnostic()->execution_fault == ExecutionFault::DeadlineMissed);
    assert(session.plans().release_after_shutdown());
    assert(session.dispatch_clock(clock, sink) == ExecutionStatus::Error && clock.reads == 2);
  }
}
void in_flight_shutdown() {
  auto first = management_owned(initial_plan());
  std::weak_ptr<const Plan> live = first;
  LiveMotionSession<1> session(std::move(first), timing);
  struct BlockingSink : Sink {
    std::atomic<bool> in_write{}, proceed{};
    bool try_write(const LiveJointCommand<1> &command) noexcept {
      in_write.store(true, std::memory_order_release);
      while (!proceed.load(std::memory_order_acquire))
        std::this_thread::yield();
      return Sink::try_write(command);
    }
  } sink;
  std::atomic<bool> finish_cycle{};
  std::thread cyclic([&] {
    assert(session.dispatch(at(0), sink) == ExecutionStatus::Running);
    while (!finish_cycle.load(std::memory_order_acquire))
      std::this_thread::yield();
    assert(session.dispatch(at(1), sink) == ExecutionStatus::Inactive);
  });
  while (!sink.in_write.load(std::memory_order_acquire))
    std::this_thread::yield();
  assert(session.plans().request_shutdown());
  assert(!session.plans().release_after_shutdown() && !live.expired());
  sink.proceed.store(true, std::memory_order_release);
  // The in-flight write can complete; the next dispatch acknowledges shutdown.
  finish_cycle.store(true, std::memory_order_release);
  while (session.plans().state() != LiveSessionState::Quiescent)
    std::this_thread::yield();
  assert(session.plans().release_after_shutdown() && live.expired());
  cyclic.join();
  assert(sink.accepted == 1 && sink.attempts == 1 && sink.stops == 1);
}
void concurrent_switches() {
  auto constant = [](std::uint64_t generation, std::uint64_t parent, std::uint64_t start) {
    return management_owned(
        prepare({{0}, {0}, {0}, at(start).seconds}, 0, {generation, parent, start, 7}, .2));
  };
  LiveMotionSession<1> session(constant(1, 0, 0), timing);
  Sink sink;
  std::atomic<std::uint64_t> permitted_tick{};
  std::thread cyclic([&] {
    for (std::uint64_t tick = 0;; ++tick) {
      while (tick > permitted_tick.load(std::memory_order_acquire))
        std::this_thread::yield();
      const auto status = session.dispatch(at(tick), sink);
      if (status == ExecutionStatus::Inactive)
        break;
      assert(status == ExecutionStatus::Running);
    }
  });
  for (std::uint64_t i = 1; i <= 200; ++i) {
    auto plan = constant(i + 1, i, i * 4);
    std::weak_ptr<const Plan> pending = plan;
    assert(session.plans().try_offer(plan));
    plan.reset();
    permitted_tick.store(i * 4, std::memory_order_release);
    std::optional<LivePlanResponse> response;
    while (!(response = session.plans().collect_response())) {
      const auto snapshot = session.snapshot();
      assert(!snapshot ||
             (!snapshot->quiescent && snapshot->generation >= i && snapshot->generation <= i + 1));
      std::this_thread::yield();
    }
    assert(response->status == LiveResponseStatus::Activated && response->generation == i + 1 &&
           response->retired_generation == i && !pending.expired());
    assert(session.plans().retained_plan_count() == 1);
  }
  assert(session.plans().request_shutdown());
  permitted_tick.store(801, std::memory_order_release);
  while (session.plans().state() != LiveSessionState::Quiescent)
    std::this_thread::yield();
  assert(session.plans().release_after_shutdown());
  cyclic.join();
  assert(!session.diagnostic() && sink.accepted == 801 && sink.stops == 1);
}
void offer_racing_with_fault() {
  for (int run = 0; run < 200; ++run) {
    auto first = management_owned(initial_plan());
    auto next = management_owned(replacement(*first, 20));
    std::weak_ptr<const Plan> active = first, candidate = next;
    LiveMotionSession<1> session(std::move(first), timing);
    Sink sink;
    std::atomic<bool> start{}, released{};
    std::thread cyclic([&] {
      while (!start.load(std::memory_order_acquire))
        std::this_thread::yield();
      assert(session.dispatch({10.002, 7, 0}, sink) == ExecutionStatus::Error);
      while (!released.load(std::memory_order_acquire))
        std::this_thread::yield();
      assert(session.dispatch(at(0), sink) == ExecutionStatus::Error);
    });
    start.store(true, std::memory_order_release);
    if (run % 2)
      std::this_thread::yield();
    const auto offer = session.plans().try_offer(next);
    assert(offer.status == LivePlanOfferStatus::Offered || offer.status == LivePlanOfferStatus::Stopping);
    next.reset();
    while (session.plans().state() != LiveSessionState::Quiescent) {
      if (const auto diagnostic = session.diagnostic())
        assert(diagnostic->code == LiveSessionFaultCode::Execution && diagnostic->generation == 1 &&
               diagnostic->tick == 0 && diagnostic->execution_fault == ExecutionFault::DeadlineMissed);
      std::this_thread::yield();
    }
    // An offer can cross publication after the RT fault has already detached its
    // caches. Quiescence retires it even when there is no individual response.
    if (const auto response = session.plans().collect_response())
      assert(response->status == LiveResponseStatus::Faulted ||
             response->status == LiveResponseStatus::Shutdown);
    assert(session.plans().release_after_shutdown() && active.expired() && candidate.expired());
    released.store(true, std::memory_order_release);
    cyclic.join();
    assert(sink.attempts == 0 && sink.stops == 1 && session.diagnostic()->tick == 0);
  }
}
} // namespace
int main() {
  static_assert(!std::is_copy_constructible_v<LiveMotionSession<1>>);
  static_assert(!std::is_move_constructible_v<LiveMotionSession<1>>);
  bool rejected = false;
  try {
    LiveMotionSession<1> invalid({}, timing);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  assert(rejected);
  retirement_and_completion();
  cancellation_and_backpressure();
  shutdown_outstanding();
  faults_and_terminal_refusal();
  in_flight_shutdown();
  concurrent_switches();
  offer_racing_with_fault();
  std::cout << "Owned cache retirement, cancellation, sink commit/faults, terminal delivery, shutdown and "
               "200 two-thread switches / fault-offer races passed\n";
}
