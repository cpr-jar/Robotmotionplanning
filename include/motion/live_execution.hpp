#pragma once
#include "execution.hpp"
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
template <std::size_t N> class PreparedLivePlan {
public:
  const LivePlanStamp &stamp() const { return stamp_; }
  double epoch() const { return epoch_; }
  double cycle_s() const { return cycle_; }
  std::uint64_t end_tick() const { return stamp_.start_tick + commands_.size() - 1; }
  const JointLimits &joint_limits() const { return limits_; }
  const FixedJointCommand<N> &command(std::uint64_t tick) const {
    return commands_[tick - stamp_.start_tick];
  }

private:
  template <std::size_t M>
  friend LivePlanResult<M> prepare_live_plan(std::shared_ptr<const IExecutionSchedule>, double, LivePlanStamp,
                                             std::size_t);
  PreparedLivePlan() = default;
  LivePlanStamp stamp_;
  double epoch_{}, cycle_{};
  JointLimits limits_;
  std::vector<FixedJointCommand<N>> commands_;
};
template <std::size_t N>
LivePlanResult<N> prepare_live_plan(std::shared_ptr<const IExecutionSchedule> schedule, double epoch,
                                    LivePlanStamp stamp, std::size_t capacity) {
  try {
    if (!schedule || !stamp.clock || !stamp.generation || stamp.generation <= stamp.parent_generation ||
        schedule->initial_state().q.size() != N || schedule->total_ticks() >= capacity ||
        schedule->total_ticks() > schedule_detail::max_ticks ||
        stamp.start_tick > schedule_detail::max_ticks - schedule->total_ticks() ||
        !schedule_detail::valid_clock(epoch, schedule->cycle_s(), stamp.start_tick + schedule->total_ticks()))
      return {{}, "Invalid live schedule/stamp/capacity"};
    if (schedule->initial_state().timestamp_s !=
        epoch + schedule_detail::tick_time(stamp.start_tick, schedule->cycle_s()))
      return {{}, "Live activation timestamp mismatch"};
    const auto &boundaries = schedule->boundary_ticks();
    if (!std::is_sorted(boundaries.begin(), boundaries.end()) ||
        (!boundaries.empty() && boundaries.back() > schedule->total_ticks()))
      return {{}, "Invalid live rest markers"};
    auto p = std::shared_ptr<PreparedLivePlan<N>>(new PreparedLivePlan<N>);
    p->stamp_ = stamp;
    p->epoch_ = epoch;
    p->cycle_ = schedule->cycle_s();
    p->limits_ = schedule->joint_limits();
    for (std::uint64_t tick = 0; tick <= schedule->total_ticks(); ++tick) {
      const auto state = schedule->sample_tick(tick);
      if (!within_joint_limits(state, p->limits_) || !std::isfinite(state.timestamp_s) ||
          state.timestamp_s != schedule->initial_state().timestamp_s + schedule->tick_time(tick) ||
          (std::binary_search(boundaries.begin(), boundaries.end(), tick) && !is_rest(state, 0)))
        return {{}, "Invalid live reference state"};
      FixedJointCommand<N> c;
      std::copy(state.q.begin(), state.q.end(), c.q.begin());
      std::copy(state.dq.begin(), state.dq.end(), c.dq.begin());
      std::copy(state.ddq.begin(), state.ddq.end(), c.ddq.begin());
      c.tick = stamp.start_tick + tick;
      c.program_time_s = schedule_detail::tick_time(c.tick, p->cycle_);
      c.timestamp_s = epoch + c.program_time_s;
      c.terminal = tick == schedule->total_ticks();
      if (c.terminal && !buffer_detail::rest(c))
        return {{}, "Live plans need terminal rest"};
      p->commands_.push_back(c);
    }
    return {std::move(p), "Immutable admitted-reference cache prepared"};
  } catch (const std::exception &e) {
    return {{}, e.what()};
  }
}
enum class LiveResponseStatus { Activated, IdentityMismatch, StateMismatch, Late, Committed, Faulted };
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
};
template <std::size_t N> class LiveMotionExecutor {
  static_assert(std::atomic<std::uint64_t>::is_always_lock_free);

public:
  LiveMotionExecutor(const PreparedLivePlan<N> &initial, LivePlanMailbox<N> &mailbox,
                     ExecutionTimingPolicy policy, std::uint64_t lead = 1)
      : active_(&initial), mailbox_(mailbox), policy_(policy), lead_(lead) {
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
      const auto before = version_.load(std::memory_order_acquire);
      if (before & 1)
        continue;
      LiveExecutionSnapshot s{generation_.load(std::memory_order_relaxed),
                              tick_.load(std::memory_order_relaxed),
                              committed_.load(std::memory_order_relaxed),
                              ExecutionFault(fault_code_.load(std::memory_order_relaxed)),
                              bool(finished_code_.load(std::memory_order_relaxed))};
      std::atomic_thread_fence(std::memory_order_acquire);
      if (version_.load(std::memory_order_relaxed) == before)
        return s;
    }
    return std::nullopt;
  }
  ExecutionFault fault() const noexcept { return fault_; }
  std::uint64_t generation() const noexcept { return active_->stamp().generation; }
  void service_requests() noexcept {
    const auto *p = mailbox_.offered();
    if (!p || p == pending_)
      return;
    if (fault_ != ExecutionFault::None || finished_) {
      respond(*p, LiveResponseStatus::Faulted);
      return;
    }
    const auto &s = p->stamp();
    const auto &a = active_->stamp();
    const auto &x = p->joint_limits();
    const auto &y = active_->joint_limits();
    if (s.parent_generation != a.generation || s.generation <= a.generation || s.clock != a.clock ||
        p->epoch() != active_->epoch() || p->cycle_s() != active_->cycle_s() ||
        x.min_position != y.min_position || x.max_position != y.max_position ||
        x.max_velocity != y.max_velocity || x.max_acceleration != y.max_acceleration ||
        x.max_jerk != y.max_jerk) {
      respond(*p, LiveResponseStatus::IdentityMismatch);
      return;
    }
    if (s.start_tick < next_ || s.start_tick - next_ < lead_ || s.start_tick > active_->end_tick()) {
      respond(*p, LiveResponseStatus::Late);
      return;
    }
    if (fence_ != UINT64_MAX && s.start_tick <= fence_) {
      respond(*p, LiveResponseStatus::Committed);
      return;
    }
    const auto &old = active_->command(s.start_tick);
    const auto &fresh = p->command(s.start_tick);
    if (old.q != fresh.q || old.dq != fresh.dq || old.ddq != fresh.ddq ||
        old.timestamp_s != fresh.timestamp_s) {
      respond(*p, LiveResponseStatus::StateMismatch);
      return;
    }
    pending_ = p;
  }
  bool commit_delivery(std::uint64_t generation, std::uint64_t through) noexcept {
    if (fault_ != ExecutionFault::None || generation != active_->stamp().generation ||
        through > active_->end_tick() || (fence_ != UINT64_MAX && through < fence_))
      return false;
    fence_ = through;
    if (pending_ && pending_->stamp().start_tick <= through) {
      const auto *p = pending_;
      pending_ = nullptr;
      respond(*p, LiveResponseStatus::Committed);
    }
    publish();
    return true;
  }
  ExecutionStatus poll(TrackingTime now, LiveJointCommand<N> &out) noexcept {
    return poll_impl(now, out, [now] { return now; }, false);
  }
  template <class Clock> ExecutionStatus poll_clock(Clock &clock, LiveJointCommand<N> &out) noexcept {
    return poll_impl(clock.now(), out, [&clock] { return clock.now(); }, true);
  }

private:
  void publish() noexcept {
    version_.fetch_add(1, std::memory_order_acq_rel);
    std::atomic_thread_fence(std::memory_order_release);
    generation_.store(active_->stamp().generation, std::memory_order_relaxed);
    tick_.store(next_, std::memory_order_relaxed);
    committed_.store(fence_, std::memory_order_relaxed);
    fault_code_.store(std::uint64_t(fault_), std::memory_order_relaxed);
    finished_code_.store(finished_, std::memory_order_relaxed);
    version_.fetch_add(1, std::memory_order_release);
  }
  void respond(const PreparedLivePlan<N> &p, LiveResponseStatus status, std::uint64_t retired = 0) noexcept {
    mailbox_.answer({status, p.stamp().generation, retired, p.stamp().start_tick});
  }
  ExecutionStatus fail(ExecutionFault fault) noexcept {
    fault_ = fault;
    if (pending_) {
      auto *p = pending_;
      pending_ = nullptr;
      respond(*p, LiveResponseStatus::Faulted);
    }
    publish();
    return ExecutionStatus::Error;
  }
  bool clock_valid(TrackingTime t) noexcept {
    if (!std::isfinite(t.seconds) || !std::isfinite(t.uncertainty_s) || t.uncertainty_s < 0 ||
        t.clock != active_->stamp().clock) {
      fail(ExecutionFault::InvalidClock);
      return false;
    }
    if (t.uncertainty_s > policy_.max_clock_uncertainty_s) {
      fail(ExecutionFault::ExcessUncertainty);
      return false;
    }
    if (last_ && t.seconds < *last_) {
      fail(ExecutionFault::ClockRegression);
      return false;
    }
    last_ = t.seconds;
    return true;
  }
  template <class Observe>
  ExecutionStatus poll_impl(TrackingTime now, LiveJointCommand<N> &out, Observe observe,
                            bool second) noexcept {
    service_requests();
    if (fault_ != ExecutionFault::None)
      return ExecutionStatus::Error;
    if (finished_)
      return ExecutionStatus::Inactive;
    if (!clock_valid(now))
      return ExecutionStatus::Error;
    const double due = active_->epoch() + schedule_detail::tick_time(next_, active_->cycle_s());
    const auto valid = [&](TrackingTime t) {
      return static_cast<long double>(t.seconds) - t.uncertainty_s >=
                 static_cast<long double>(due) - policy_.max_early_s &&
             static_cast<long double>(t.seconds) + t.uncertainty_s + policy_.dispatch_latency_bound_s <=
                 static_cast<long double>(due) + policy_.max_late_s;
    };
    if (static_cast<long double>(now.seconds) + now.uncertainty_s + policy_.dispatch_latency_bound_s >
        static_cast<long double>(due) + policy_.max_late_s)
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
    const auto *retired = active_;
    const bool activate = pending_ && pending_->stamp().start_tick == next_;
    if (activate) {
      active_ = pending_;
      pending_ = nullptr;
    }
    out = {active_->command(next_), active_->stamp().generation};
    ++next_;
    finished_ = out.joints.terminal;
    publish();
    if (activate)
      respond(*active_, LiveResponseStatus::Activated, retired->stamp().generation);
    return finished_ ? ExecutionStatus::Finished : ExecutionStatus::Running;
  }
  const PreparedLivePlan<N> *active_;
  const PreparedLivePlan<N> *pending_{};
  LivePlanMailbox<N> &mailbox_;
  ExecutionTimingPolicy policy_;
  std::uint64_t lead_, next_{}, fence_{UINT64_MAX};
  std::optional<double> last_;
  ExecutionFault fault_{ExecutionFault::None};
  bool finished_{};
  std::atomic<std::uint64_t> version_{}, generation_{}, tick_{}, committed_{UINT64_MAX}, fault_code_{},
      finished_code_{};
};
} // namespace motion
