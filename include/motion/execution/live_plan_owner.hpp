#pragma once
#include "motion/execution/live_motion_executor.hpp"
namespace motion {
enum class LiveSessionState : std::uint64_t { Running, ShutdownRequested, Quiescent };
enum class LivePlanOfferStatus { Offered, Busy, InvalidPlan, GenerationReused, Stopping };
struct LivePlanOfferResult {
  LivePlanOfferStatus status{LivePlanOfferStatus::InvalidPlan};
  std::uint64_t generation{};
  explicit operator bool() const noexcept { return status == LivePlanOfferStatus::Offered; }
};
template <std::size_t N> class LiveMotionSession;
/// Non-RT endpoint, owned by a LiveMotionSession. One management thread calls
/// try_offer/collect_response/request_cancel/release_after_shutdown and owns all
/// shared_ptr operations. request_shutdown() and state() are safe from any thread.
template <std::size_t N> class LivePlanOwner {
public:
  LivePlanOwner(const LivePlanOwner &) = delete;
  LivePlanOwner &operator=(const LivePlanOwner &) = delete;

  LivePlanOfferResult try_offer(const std::shared_ptr<const PreparedLivePlan<N>> &plan) {
    if (state() != LiveSessionState::Running)
      return {LivePlanOfferStatus::Stopping, plan ? plan->stamp().generation : 0};
    if (!plan)
      return {LivePlanOfferStatus::InvalidPlan, 0};
    const auto generation = plan->stamp().generation;
    if (offered_plan_)
      return {LivePlanOfferStatus::Busy, generation};
    if (generation <= last_offered_generation_)
      return {LivePlanOfferStatus::GenerationReused, generation};
    offered_plan_ = plan; // Retain before publishing a borrowed address to RT.
    if (!mailbox_.try_offer(*offered_plan_)) {
      offered_plan_.reset();
      return {LivePlanOfferStatus::Busy, generation};
    }
    last_offered_generation_ = generation;
    return {LivePlanOfferStatus::Offered, generation};
  }
  std::optional<LivePlanResponse> collect_response() {
    LivePlanResponse response;
    if (!mailbox_.try_response(response))
      return std::nullopt;
    if (response.status == LiveResponseStatus::Activated)
      active_plan_ = std::move(offered_plan_); // Retire the old cache on this thread.
    else
      offered_plan_.reset();
    return response;
  }
  /// Best-effort request, not an acknowledgement. The definitive reply may still
  /// be Activated if the cyclic thread has already crossed the activation tick.
  bool request_cancel() noexcept {
    if (state() != LiveSessionState::Running || !offered_plan_)
      return false;
    cancellation_generation_.store(offered_plan_->stamp().generation, std::memory_order_release);
    return true;
  }
  /// Asynchronous. An in-progress dispatch can finish; wait for Quiescent.
  bool request_shutdown() noexcept {
    auto expected = LiveSessionState::Running;
    return state_.compare_exchange_strong(expected, LiveSessionState::ShutdownRequested,
                                          std::memory_order_acq_rel, std::memory_order_acquire);
  }
  LiveSessionState state() const noexcept { return state_.load(std::memory_order_acquire); }
  /// Only after RT acknowledgement; does not wait. Still join the RT thread
  /// before destroying the session itself. Further dispatch calls are harmless.
  bool release_after_shutdown() {
    if (state() != LiveSessionState::Quiescent)
      return false;
    // Quiescence retires every cache, including an offer racing with a fault.
    // No runtime operation will access the mailbox again.
    LivePlanResponse ignored;
    mailbox_.try_response(ignored);
    offered_plan_.reset();
    active_plan_.reset();
    return true;
  }
  /// Management-thread getters, not synchronized observer snapshots.
  std::size_t retained_plan_count() const noexcept {
    return std::size_t(bool(active_plan_)) + std::size_t(bool(offered_plan_));
  }
  std::uint64_t last_offered_generation() const noexcept { return last_offered_generation_; }

private:
  friend class LiveMotionSession<N>;
  explicit LivePlanOwner(std::shared_ptr<const PreparedLivePlan<N>> initial)
      : active_plan_(std::move(initial)) {
    if (!active_plan_)
      throw std::invalid_argument("Missing initial live plan");
    last_offered_generation_ = active_plan_->stamp().generation;
  }
  std::shared_ptr<const PreparedLivePlan<N>> active_plan_, offered_plan_;
  LivePlanMailbox<N> mailbox_;
  std::atomic<LiveSessionState> state_{LiveSessionState::Running};
  std::atomic<std::uint64_t> cancellation_generation_{};
  std::uint64_t last_offered_generation_{};
};
} // namespace motion
