#pragma once
#include "motion/execution/cycle_aligned_program.hpp"
#include <atomic>
namespace motion {
template <std::size_t N> struct FixedJointCommand {
  std::array<double, N> q{}, dq{}, ddq{};
  double timestamp_s{}, program_time_s{};
  std::uint64_t tick{};
  bool terminal{};
  bool operator==(const FixedJointCommand &) const = default;
};
namespace buffer_detail {
template <std::size_t N> bool finite(const FixedJointCommand<N> &c) {
  if (!std::isfinite(c.timestamp_s) || !std::isfinite(c.program_time_s))
    return false;
  for (std::size_t i = 0; i < N; ++i)
    if (!std::isfinite(c.q[i]) || !std::isfinite(c.dq[i]) || !std::isfinite(c.ddq[i]))
      return false;
  return true;
}
template <std::size_t N> bool rest(const FixedJointCommand<N> &c) {
  for (std::size_t i = 0; i < N; ++i)
    if (c.dq[i] != 0 || c.ddq[i] != 0)
      return false;
  return true;
}
} // namespace buffer_detail
/// Exactly one producer and one consumer; overflow returns false without replacing unread data.
template <std::size_t N, std::size_t Capacity> class SpscSetpointBuffer {
  static_assert(Capacity >= 2);
  static_assert(std::atomic<std::size_t>::is_always_lock_free);

public:
  bool try_push(const FixedJointCommand<N> &c) noexcept {
    const auto w = write_.load(std::memory_order_relaxed), next = (w + 1) % (Capacity + 1);
    if (next == read_.load(std::memory_order_acquire))
      return false;
    data_[w] = c;
    write_.store(next, std::memory_order_release);
    return true;
  }
  bool try_pop(FixedJointCommand<N> &c) noexcept {
    const auto r = read_.load(std::memory_order_relaxed);
    if (r == write_.load(std::memory_order_acquire))
      return false;
    c = data_[r];
    read_.store((r + 1) % (Capacity + 1), std::memory_order_release);
    return true;
  }

private:
  std::array<FixedJointCommand<N>, Capacity + 1> data_{};
  alignas(64) std::atomic<std::size_t> read_{};
  alignas(64) std::atomic<std::size_t> write_{};
};
enum class BufferFillStatus { Ready, Finished, Error };
struct BufferFillResult {
  BufferFillStatus status;
  std::size_t count{};
};
/// Samples vector-valued trajectories outside the realtime thread; fill() may allocate.
template <std::size_t N> class BufferedMotionProducer {
public:
  BufferedMotionProducer(std::shared_ptr<const IExecutionSchedule> schedule, double cycle)
      : schedule_(std::move(schedule)) {
    if (!schedule_ || schedule_->initial_state().q.size() != N || schedule_->cycle_s() != cycle)
      throw std::invalid_argument("Producer schedule mismatch");
    initial_ = make(0);
  }
  const FixedJointCommand<N> &initial_command() const { return initial_; }
  template <std::size_t Capacity>
  BufferFillResult fill(SpscSetpointBuffer<N, Capacity> &ring, std::size_t budget) {
    std::size_t count = 0;
    try {
      while (count < budget && next_ <= schedule_->total_ticks()) {
        auto c = make(next_);
        if (!ring.try_push(c))
          break;
        ++next_;
        ++count;
      }
      return {next_ > schedule_->total_ticks() ? BufferFillStatus::Finished : BufferFillStatus::Ready, count};
    } catch (...) {
      return {BufferFillStatus::Error, count};
    }
  }

private:
  FixedJointCommand<N> make(std::uint64_t tick) const {
    const auto s = schedule_->sample_tick(tick);
    if (s.q.size() != N || !within_joint_limits(s, schedule_->joint_limits()) ||
        s.timestamp_s != schedule_->initial_state().timestamp_s + schedule_->tick_time(tick))
      throw std::domain_error("Invalid generated command");
    FixedJointCommand<N> c;
    std::copy(s.q.begin(), s.q.end(), c.q.begin());
    std::copy(s.dq.begin(), s.dq.end(), c.dq.begin());
    std::copy(s.ddq.begin(), s.ddq.end(), c.ddq.begin());
    c.tick = tick;
    c.timestamp_s = s.timestamp_s;
    c.program_time_s = schedule_->tick_time(tick);
    c.terminal = tick == schedule_->total_ticks();
    return c;
  }
  std::shared_ptr<const IExecutionSchedule> schedule_;
  FixedJointCommand<N> initial_;
  std::uint64_t next_{};
};
} // namespace motion
