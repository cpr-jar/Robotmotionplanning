#include "motion/live_execution.hpp"
#include "motion/polynomial_schedule.hpp"
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <new>
#if defined(__GNUC__) || defined(__clang__)
#define MOTION_NOINLINE __attribute__((noinline))
#else
#define MOTION_NOINLINE
#endif
namespace {
bool track = false;
std::size_t allocations{}, deletions{};
} // namespace
MOTION_NOINLINE void *operator new(std::size_t n) {
  if (track)
    ++allocations;
  if (auto *p = std::malloc(n ? n : 1))
    return p;
  throw std::bad_alloc();
}
MOTION_NOINLINE void *operator new[](std::size_t n) {
  return ::operator new(n);
}
MOTION_NOINLINE void operator delete(void *p) noexcept {
  if (track && p)
    ++deletions;
  std::free(p);
}
MOTION_NOINLINE void operator delete[](void *p) noexcept {
  ::operator delete(p);
}
MOTION_NOINLINE void operator delete(void *p, std::size_t) noexcept {
  ::operator delete(p);
}
MOTION_NOINLINE void operator delete[](void *p, std::size_t) noexcept {
  ::operator delete(p);
}
MOTION_NOINLINE void *operator new(std::size_t n, std::align_val_t a) {
  if (track)
    ++allocations;
  const auto alignment = std::size_t(a), size = ((n ? n : 1) + alignment - 1) / alignment * alignment;
  if (auto *p = std::aligned_alloc(alignment, size))
    return p;
  throw std::bad_alloc();
}
MOTION_NOINLINE void *operator new[](std::size_t n, std::align_val_t a) {
  return ::operator new(n, a);
}
MOTION_NOINLINE void operator delete(void *p, std::align_val_t) noexcept {
  ::operator delete(p);
}
MOTION_NOINLINE void operator delete[](void *p, std::align_val_t) noexcept {
  ::operator delete(p);
}
MOTION_NOINLINE void operator delete(void *p, std::size_t, std::align_val_t) noexcept {
  ::operator delete(p);
}
MOTION_NOINLINE void operator delete[](void *p, std::size_t, std::align_val_t) noexcept {
  ::operator delete(p);
}
int main() {
  using namespace motion;
  JointLimits limits{{-2}, {2}, {2}, {5}, {50}};
  auto polynomial = plan_boundary_quintic(limits, {{0}, {0}, {0}, 10}, {{1}, {0}, {0}}, 2);
  assert(polynomial);
  auto schedule = std::make_shared<BoundaryPolynomialSchedule>(polynomial.trajectory, .01);
  const auto first = prepare_live_plan<1>(schedule, 10, {1, 0, 0, 7});
  assert(first);
  const auto boundary = first.plan->command(50);
  auto changed = plan_boundary_quintic(
      limits, {{boundary.q[0]}, {boundary.dq[0]}, {boundary.ddq[0]}, boundary.timestamp_s}, {{.3}, {0}, {0}},
      2);
  assert(changed);
  auto next_schedule = std::make_shared<BoundaryPolynomialSchedule>(changed.trajectory, .01);
  const auto next = prepare_live_plan<1>(next_schedule, 10, {2, 1, 50, 7});
  assert(next);
  LivePlanMailbox<1> box;
  LiveMotionExecutor<1> execution(*first.plan, box, {0, .001, 0, 0});
  LiveJointCommand<1> out;
  assert(box.try_offer(*next.plan));
  std::size_t switches = 0;
  track = true;
  for (std::uint64_t tick = 0; tick <= next.plan->end_tick(); ++tick) {
    const auto status = execution.poll({10 + schedule_detail::tick_time(tick, .01), 7, 0}, out);
    assert(status == ExecutionStatus::Running || status == ExecutionStatus::Finished);
    assert(execution.commit_delivery(out.generation, tick));
    const auto snapshot = execution.snapshot();
    assert(snapshot && snapshot->next_tick == tick + 1);
    LivePlanResponse response;
    if (box.try_response(response)) {
      assert(response.status == LiveResponseStatus::Activated);
      ++switches;
    }
  }
  track = false;
  assert(switches == 1 && allocations == 0 && deletions == 0);
  std::cout
      << "No C++ new/delete during live activation, cached dispatch, snapshot, fence or acknowledgement\n";
}
