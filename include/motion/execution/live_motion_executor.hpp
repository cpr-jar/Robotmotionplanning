#pragma once
#include "motion/execution/deadline_motion_executor.hpp"
#include <atomic>
namespace motion {
struct LivePlanStamp {
  std::uint64_t generation{}, parent_generation{}, start_tick{};
  ClockId clock{};
};
template <std::size_t N> struct LiveJointCommand {
  FixedJointCommand<N> joints;
  std::uint64_t generation{};
};
template <std::size_t N> class PreparedLivePlan;
template <std::size_t N> struct LivePlanResult {
  std::shared_ptr<const PreparedLivePlan<N>> plan;
  std::string diagnostic;
  explicit operator bool() const { return bool(plan); }
};
template <std::size_t N>
LivePlanResult<N> prepare_live_plan(std::shared_ptr<const IExecutionSchedule>, double, LivePlanStamp,
                                    std::size_t = 200000);
/// Immutable cache owned by the application; the executor borrows its address.
template <std::size_t N> class PreparedLivePlan {
public:
  const LivePlanStamp &stamp() const { return stamp_; }
  double epoch() const { return execution_epoch_s_; }
  double cycle_s() const { return cycle_period_s_; }
  std::uint64_t end_tick() const { return stamp_.start_tick + cached_commands_.size() - 1; }
  const JointLimits &joint_limits() const { return joint_limits_; }
  /// Precondition: stamp().start_tick <= tick <= end_tick(); no range check on this fast path.
  const FixedJointCommand<N> &command(std::uint64_t tick) const {
    return cached_commands_[tick - stamp_.start_tick];
  }

private:
  template <std::size_t M>
  friend LivePlanResult<M> prepare_live_plan(std::shared_ptr<const IExecutionSchedule>, double, LivePlanStamp,
                                             std::size_t);
  PreparedLivePlan() = default;
  LivePlanStamp stamp_;
  double execution_epoch_s_{}, cycle_period_s_{};
  JointLimits joint_limits_;
  std::vector<FixedJointCommand<N>> cached_commands_;
};
template <std::size_t N>
LivePlanResult<N> prepare_live_plan(std::shared_ptr<const IExecutionSchedule> schedule, double epoch,
                                    LivePlanStamp stamp, std::size_t capacity) {
  try {
    if (!schedule || !stamp.clock || !stamp.generation || stamp.generation <= stamp.parent_generation ||
        schedule->total_ticks() >= capacity || schedule->total_ticks() > schedule_detail::max_ticks ||
        stamp.start_tick > schedule_detail::max_ticks - schedule->total_ticks() ||
        !schedule_detail::valid_clock(epoch, schedule->cycle_s(), stamp.start_tick + schedule->total_ticks()))
      return {{}, "Invalid live schedule/stamp/capacity"};
    const auto initial = schedule->initial_state();
    const auto limits = schedule->joint_limits();
    const auto ticks = schedule->total_ticks();
    const double cycle = schedule->cycle_s();
    check_limits(limits, N);
    check_state(initial, N);
    if (!within_joint_limits(initial, limits))
      return {{}, "Invalid live initial state"};
    if (initial.timestamp_s != epoch + schedule_detail::tick_time(stamp.start_tick, cycle))
      return {{}, "Live activation timestamp mismatch"};
    if (schedule->duration() != schedule_detail::tick_time(ticks, cycle))
      return {{}, "Invalid live schedule duration"};
    const auto &boundaries = schedule->boundary_ticks();
    if (!std::is_sorted(boundaries.begin(), boundaries.end()) ||
        (!boundaries.empty() && boundaries.back() > schedule->total_ticks()))
      return {{}, "Invalid live rest markers"};
    auto p = std::shared_ptr<PreparedLivePlan<N>>(new PreparedLivePlan<N>);
    p->stamp_ = stamp;
    p->execution_epoch_s_ = epoch;
    p->cycle_period_s_ = cycle;
    p->joint_limits_ = limits;
    p->cached_commands_.reserve(ticks + 1);
    for (std::uint64_t tick = 0; tick <= ticks; ++tick) {
      const auto state = schedule->sample_tick(tick);
      check_state(state, N); // Validate the provider before copying to fixed-size arrays.
      const double local_time = schedule_detail::tick_time(tick, cycle);
      if (!within_joint_limits(state, p->joint_limits_) || !std::isfinite(state.timestamp_s) ||
          schedule->tick_time(tick) != local_time || state.timestamp_s != initial.timestamp_s + local_time ||
          (tick == 0 && (state.q != initial.q || state.dq != initial.dq || state.ddq != initial.ddq)) ||
          (std::binary_search(boundaries.begin(), boundaries.end(), tick) && !is_rest(state, 0)))
        return {{}, "Invalid live reference state"};
      FixedJointCommand<N> c;
      std::copy(state.q.begin(), state.q.end(), c.q.begin());
      std::copy(state.dq.begin(), state.dq.end(), c.dq.begin());
      std::copy(state.ddq.begin(), state.ddq.end(), c.ddq.begin());
      c.tick = stamp.start_tick + tick;
      c.program_time_s = schedule_detail::tick_time(c.tick, p->cycle_period_s_);
      c.timestamp_s = epoch + c.program_time_s;
      c.terminal = tick == ticks;
      if (c.terminal && !buffer_detail::rest(c))
        return {{}, "Live plans need terminal rest"};
      p->cached_commands_.push_back(c);
    }
    return {std::move(p), "Immutable admitted-reference cache prepared"};
  } catch (const std::exception &e) {
    return {{}, e.what()};
  }
}
enum class LiveResponseStatus {
  Activated,
  IdentityMismatch,
  StateMismatch,
  Late,
  Committed,
  Faulted,
  Cancelled,
  Shutdown
};
struct LivePlanResponse {
  LiveResponseStatus status{};
  std::uint64_t generation{}, retired_generation{}, activation_tick{};
};
template <std::size_t N> class LivePlanMailbox {
public:
  bool try_offer(const PreparedLivePlan<N> &p) noexcept {
    if (state_.load(std::memory_order_acquire) != State::Idle)
      return false;
    request_ = &p;
    state_.store(State::Offered, std::memory_order_release);
    return true;
  }
  bool try_response(LivePlanResponse &r) noexcept {
    if (state_.load(std::memory_order_acquire) != State::Responded)
      return false;
    r = response_;
    state_.store(State::Idle, std::memory_order_release);
    return true;
  }

private:
  template <std::size_t> friend class LiveMotionExecutor;
  enum class State { Idle, Offered, Responded };
  const PreparedLivePlan<N> *offered() noexcept {
    return state_.load(std::memory_order_acquire) == State::Offered ? request_ : nullptr;
  }
  void answer(LivePlanResponse r) noexcept {
    response_ = r;
    state_.store(State::Responded, std::memory_order_release);
  }
  std::atomic<State> state_{State::Idle};
  const PreparedLivePlan<N> *request_{};
  LivePlanResponse response_{};
};
struct LiveExecutionSnapshot {
  std::uint64_t generation{}, next_tick{}, committed_through{UINT64_MAX};
  ExecutionFault fault{};
  bool finished{};
  bool quiescent{};
};
/// Single cyclic owner; only snapshot() is intended for concurrent observer reads.
/// Keep active/pending caches alive until a matching rejection/retirement response.
template <std::size_t N> class LiveMotionExecutor {
  static_assert(std::atomic<std::uint64_t>::is_always_lock_free);

public:
  LiveMotionExecutor(const PreparedLivePlan<N> &initial, LivePlanMailbox<N> &mailbox,
                     ExecutionTimingPolicy policy, std::uint64_t lead = 1)
      : active_plan_(&initial), plan_mailbox_(mailbox), timing_policy_(policy), minimum_lead_ticks_(lead) {
    if (initial.stamp().start_tick != 0 || initial.stamp().parent_generation != 0 || !lead)
      throw std::invalid_argument("Invalid initial live identity");
    for (double x : {policy.max_early_s, policy.max_late_s, policy.max_clock_uncertainty_s,
                     policy.dispatch_latency_bound_s})
      if (!std::isfinite(x) || x < 0)
        throw std::invalid_argument("Invalid live timing policy");
    const double window = policy.max_early_s + policy.max_late_s;
    if (window <= 0 || window >= initial.cycle_s() ||
        2 * policy.max_clock_uncertainty_s + policy.dispatch_latency_bound_s > window)
      throw std::invalid_argument("Invalid live dispatch windows");
    publish();
  }
  std::optional<LiveExecutionSnapshot> snapshot() const noexcept {
    for (int retry = 0; retry < 3; ++retry) {
      const auto before = snapshot_version_.load(std::memory_order_acquire);
      if (before & 1)
        continue;
      LiveExecutionSnapshot s{published_generation_.load(std::memory_order_relaxed),
                              published_next_tick_.load(std::memory_order_relaxed),
                              published_committed_tick_.load(std::memory_order_relaxed),
                              ExecutionFault(published_fault_code_.load(std::memory_order_relaxed)),
                              bool(published_finished_flag_.load(std::memory_order_relaxed)),
                              bool(published_quiescent_flag_.load(std::memory_order_relaxed))};
      std::atomic_thread_fence(std::memory_order_acquire);
      if (snapshot_version_.load(std::memory_order_relaxed) == before)
        return s;
    }
    return std::nullopt;
  }
  ExecutionFault fault() const noexcept { return fault_; }
  std::uint64_t generation() const noexcept { return active_plan_ ? active_plan_->stamp().generation : 0; }
  /// Cyclic owner only. Cancellation wins if processed before activation.
  bool cancel_offer(std::uint64_t generation) noexcept {
    if (quiescent_)
      return false;
    const auto *p = plan_mailbox_.offered();
    if (!p || p->stamp().generation != generation)
      return false;
    pending_plan_ = nullptr;
    respond(*p, LiveResponseStatus::Cancelled);
    return true;
  }
  /// Cyclic owner only: detach all borrowed caches; no physical stop is performed here.
  /// No further commands or plan offers are consumed after this operation.
  void shutdown() noexcept {
    if (quiescent_)
      return;
    if (const auto *p = plan_mailbox_.offered())
      respond(*p, LiveResponseStatus::Shutdown);
    pending_plan_ = nullptr;
    active_plan_ = nullptr;
    quiescent_ = true;
    publish();
  }
  void service_requests() noexcept {
    if (quiescent_)
      return;
    const auto *p = plan_mailbox_.offered();
    if (!p || p == pending_plan_)
      return;
    if (fault_ != ExecutionFault::None || finished_) {
      respond(*p, LiveResponseStatus::Faulted);
      return;
    }
    const auto &s = p->stamp();
    const auto &a = active_plan_->stamp();
    const auto &x = p->joint_limits();
    const auto &y = active_plan_->joint_limits();
    if (s.parent_generation != a.generation || s.generation <= a.generation || s.clock != a.clock ||
        p->epoch() != active_plan_->epoch() || p->cycle_s() != active_plan_->cycle_s() ||
        x.min_position != y.min_position || x.max_position != y.max_position ||
        x.max_velocity != y.max_velocity || x.max_acceleration != y.max_acceleration ||
        x.max_jerk != y.max_jerk) {
      respond(*p, LiveResponseStatus::IdentityMismatch);
      return;
    }
    if (s.start_tick < next_tick_ || s.start_tick - next_tick_ < minimum_lead_ticks_ ||
        s.start_tick > active_plan_->end_tick()) {
      respond(*p, LiveResponseStatus::Late);
      return;
    }
    if (committed_through_tick_ != UINT64_MAX && s.start_tick <= committed_through_tick_) {
      respond(*p, LiveResponseStatus::Committed);
      return;
    }
    const auto &old = active_plan_->command(s.start_tick);
    const auto &fresh = p->command(s.start_tick);
    if (old.q != fresh.q || old.dq != fresh.dq || old.ddq != fresh.ddq ||
        old.timestamp_s != fresh.timestamp_s) {
      respond(*p, LiveResponseStatus::StateMismatch);
      return;
    }
    pending_plan_ = p;
  }
  bool commit_delivery(std::uint64_t generation, std::uint64_t through) noexcept {
    if (quiescent_ || fault_ != ExecutionFault::None || generation != active_plan_->stamp().generation ||
        through > active_plan_->end_tick() ||
        (committed_through_tick_ != UINT64_MAX && through < committed_through_tick_))
      return false;
    committed_through_tick_ = through;
    if (pending_plan_ && pending_plan_->stamp().start_tick <= through) {
      const auto *p = pending_plan_;
      pending_plan_ = nullptr;
      respond(*p, LiveResponseStatus::Committed);
    }
    publish();
    return true;
  }
  ExecutionStatus poll(ClockTime now, LiveJointCommand<N> &out) noexcept {
    return poll_impl(now, out, [now] { return now; }, false);
  }
  template <class Clock> ExecutionStatus poll_clock(Clock &clock, LiveJointCommand<N> &out) noexcept {
    return poll_impl(clock.now(), out, [&clock] { return clock.now(); }, true);
  }

private:
  void publish() noexcept {
    snapshot_version_.fetch_add(1, std::memory_order_acq_rel);
    std::atomic_thread_fence(std::memory_order_release);
    published_generation_.store(generation(), std::memory_order_relaxed);
    published_next_tick_.store(next_tick_, std::memory_order_relaxed);
    published_committed_tick_.store(committed_through_tick_, std::memory_order_relaxed);
    published_fault_code_.store(std::uint64_t(fault_), std::memory_order_relaxed);
    published_finished_flag_.store(finished_, std::memory_order_relaxed);
    published_quiescent_flag_.store(quiescent_, std::memory_order_relaxed);
    snapshot_version_.fetch_add(1, std::memory_order_release);
  }
  void respond(const PreparedLivePlan<N> &p, LiveResponseStatus status, std::uint64_t retired = 0) noexcept {
    plan_mailbox_.answer({status, p.stamp().generation, retired, p.stamp().start_tick});
  }
  ExecutionStatus fail(ExecutionFault fault) noexcept {
    fault_ = fault;
    if (pending_plan_) {
      auto *p = pending_plan_;
      pending_plan_ = nullptr;
      respond(*p, LiveResponseStatus::Faulted);
    }
    publish();
    return ExecutionStatus::Error;
  }
  bool clock_valid(ClockTime t) noexcept {
    if (!std::isfinite(t.seconds) || !std::isfinite(t.uncertainty_s) || t.uncertainty_s < 0 ||
        t.clock != active_plan_->stamp().clock) {
      fail(ExecutionFault::InvalidClock);
      return false;
    }
    if (t.uncertainty_s > timing_policy_.max_clock_uncertainty_s) {
      fail(ExecutionFault::ExcessUncertainty);
      return false;
    }
    if (last_observed_time_s_ && t.seconds < *last_observed_time_s_) {
      fail(ExecutionFault::ClockRegression);
      return false;
    }
    last_observed_time_s_ = t.seconds;
    return true;
  }
  template <class Observe>
  ExecutionStatus poll_impl(ClockTime now, LiveJointCommand<N> &out, Observe observe, bool second) noexcept {
    if (quiescent_)
      return ExecutionStatus::Inactive;
    service_requests();
    if (fault_ != ExecutionFault::None)
      return ExecutionStatus::Error;
    if (finished_)
      return ExecutionStatus::Inactive;
    if (!clock_valid(now))
      return ExecutionStatus::Error;
    const double due =
        active_plan_->epoch() + schedule_detail::tick_time(next_tick_, active_plan_->cycle_s());
    const auto valid = [&](ClockTime t) {
      return static_cast<long double>(t.seconds) - t.uncertainty_s >=
                 static_cast<long double>(due) - timing_policy_.max_early_s &&
             static_cast<long double>(t.seconds) + t.uncertainty_s +
                     timing_policy_.dispatch_latency_bound_s <=
                 static_cast<long double>(due) + timing_policy_.max_late_s;
    };
    if (static_cast<long double>(now.seconds) + now.uncertainty_s + timing_policy_.dispatch_latency_bound_s >
        static_cast<long double>(due) + timing_policy_.max_late_s)
      return fail(ExecutionFault::DeadlineMissed);
    if (!valid(now))
      return ExecutionStatus::Inactive;
    if (second) {
      const auto check = observe();
      if (!clock_valid(check))
        return ExecutionStatus::Error;
      if (!valid(check))
        return fail(ExecutionFault::DeadlineMissed);
    }
    const auto *retired = active_plan_;
    const bool activate = pending_plan_ && pending_plan_->stamp().start_tick == next_tick_;
    if (activate) {
      active_plan_ = pending_plan_;
      pending_plan_ = nullptr;
    }
    out = {active_plan_->command(next_tick_), active_plan_->stamp().generation};
    ++next_tick_;
    finished_ = out.joints.terminal;
    publish();
    if (activate)
      respond(*active_plan_, LiveResponseStatus::Activated, retired->stamp().generation);
    return finished_ ? ExecutionStatus::Finished : ExecutionStatus::Running;
  }
  const PreparedLivePlan<N> *active_plan_;
  const PreparedLivePlan<N> *pending_plan_{};
  LivePlanMailbox<N> &plan_mailbox_;
  ExecutionTimingPolicy timing_policy_;
  std::uint64_t minimum_lead_ticks_, next_tick_{}, committed_through_tick_{UINT64_MAX};
  std::optional<double> last_observed_time_s_;
  ExecutionFault fault_{ExecutionFault::None};
  bool finished_{}, quiescent_{};
  std::atomic<std::uint64_t> snapshot_version_{}, published_generation_{}, published_next_tick_{},
      published_committed_tick_{UINT64_MAX}, published_fault_code_{}, published_finished_flag_{},
      published_quiescent_flag_{};
};
} // namespace motion
