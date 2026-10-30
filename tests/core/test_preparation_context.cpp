#include "motion/core/preparation_context.hpp"
#include "motion/execution/boundary_polynomial_schedule.hpp"
#include "motion/execution/live_motion_session.hpp"
#include <cassert>
#include <future>
#include <iostream>
#include <thread>
using namespace motion;
template <class F> void failure(F action, PreparationFailure code, PreparationResource resource) {
  bool caught = false;
  try {
    action();
  } catch (const PreparationError &error) {
    caught = true;
    assert(error.code() == code && error.resource() == resource && !error.phase().empty());
  }
  assert(caught);
}
class StoppingSchedule final : public IExecutionSchedule {
public:
  StoppingSchedule(std::shared_ptr<const IExecutionSchedule> source, std::stop_source &stop)
      : source_(std::move(source)), stop_(stop) {}
  double duration() const noexcept override { return source_->duration(); }
  double cycle_s() const noexcept override { return source_->cycle_s(); }
  std::uint64_t total_ticks() const noexcept override { return source_->total_ticks(); }
  double tick_time(std::uint64_t tick) const noexcept override { return source_->tick_time(tick); }
  const std::vector<std::uint64_t> &boundary_ticks() const noexcept override {
    return source_->boundary_ticks();
  }
  const JointState &initial_state() const noexcept override { return source_->initial_state(); }
  const JointLimits &joint_limits() const noexcept override { return source_->joint_limits(); }
  JointState sample(double t) const override { return source_->sample(t); }
  JointState sample_tick(std::uint64_t tick) const override {
    ++samples;
    if (tick == total_ticks())
      stop_.request_stop();
    return source_->sample_tick(tick);
  }
  mutable std::size_t samples{};

private:
  std::shared_ptr<const IExecutionSchedule> source_;
  std::stop_source &stop_;
};
int main() {
  PreparationBudget budget;
  budget.work = UINT64_MAX;
  PreparationContext maximum(budget);
  maximum.consume(PreparationResource::Work, UINT64_MAX, "maximum");
  failure([&] { maximum.consume(PreparationResource::Work, 1, "overflow"); },
          PreparationFailure::BudgetExceeded, PreparationResource::Work);
  assert(maximum.used(PreparationResource::Work) == UINT64_MAX);
  failure([&] { maximum.checkpoint("reuse"); }, PreparationFailure::BudgetExceeded,
          PreparationResource::Work);
  PreparationContext expired({}, {}, PreparationContext::Clock::now());
  failure([&] { expired.checkpoint("expired"); }, PreparationFailure::DeadlineExceeded,
          PreparationResource::Count);
  std::stop_source stop;
  PreparationContext context({}, stop.get_token());
  std::promise<void> entered, resume;
  auto ready = resume.get_future();
  std::thread worker([&] {
    context.consume(PreparationResource::Work, 1, "worker");
    entered.set_value();
    ready.wait();
    failure([&] { context.consume(PreparationResource::Work, 1, "worker"); }, PreparationFailure::Cancelled,
            PreparationResource::Count);
  });
  entered.get_future().wait();
  stop.request_stop();
  resume.set_value();
  worker.join();
  assert(context.used(PreparationResource::Work) == 1);
  const JointLimits limits{{-2}, {2}, {2}, {5}, {50}};
  const auto polynomial = plan_boundary_quintic(limits, {{0}, {0}, {0}, 0}, {{.2}, {0}, {0}}, 2);
  assert(polynomial);
  const auto schedule = std::make_shared<BoundaryPolynomialSchedule>(polynomial.trajectory, .01);
  budget = {};
  budget.cache_bytes = (schedule->total_ticks() + 1) * sizeof(FixedJointCommand<1>) - 1;
  PreparationContext too_small(budget);
  failure([&] { prepare_live_plan<1>(schedule, 0, {1, 0, 0, 7}, 1000, &too_small); },
          PreparationFailure::BudgetExceeded, PreparationResource::CacheBytes);
  budget.cache_bytes += 1;
  PreparationContext exact(budget);
  auto admitted = prepare_live_plan<1>(schedule, 0, {1, 0, 0, 7}, 1000, &exact);
  assert(admitted && exact.used(PreparationResource::CacheBytes) == budget.cache_bytes);
  // The same job cannot allocate another generation beyond its cumulative budget.
  failure([&] { prepare_live_plan<1>(schedule, 0, {2, 0, 0, 7}, 1000, &exact); },
          PreparationFailure::BudgetExceeded, PreparationResource::CacheBytes);
  assert(admitted.plan->end_tick() == 200);
  std::stop_source last_sample_stop;
  auto stopping = std::make_shared<StoppingSchedule>(schedule, last_sample_stop);
  PreparationContext last_sample({}, last_sample_stop.get_token());
  failure([&] { prepare_live_plan<1>(stopping, 0, {3, 0, 0, 7}, 1000, &last_sample); },
          PreparationFailure::Cancelled, PreparationResource::Count);
  assert(stopping->samples == 201);
  stopping->samples = 0;
  budget.cache_bytes = 0;
  PreparationContext before_sample(budget);
  failure([&] { prepare_live_plan<1>(stopping, 0, {4, 0, 0, 7}, 1000, &before_sample); },
          PreparationFailure::BudgetExceeded, PreparationResource::CacheBytes);
  assert(stopping->samples == 0);
  std::cout
      << "Preparation limits, cross-thread cancellation, deadlines and cumulative cache admission passed\n";
}
