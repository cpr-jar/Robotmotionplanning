#pragma once
#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <string_view>

namespace motion {
/// Cumulative admission counters for one preparation job, not process heap/RSS.
enum class PreparationResource {
  InputBytes,
  Assets,
  Drawings,
  Instructions,
  Work,
  SimulationTicks,
  CacheBytes,
  Count
};
enum class PreparationFailure { None, Cancelled, DeadlineExceeded, BudgetExceeded };
inline constexpr std::array<const char *, 7> preparation_resource_names{
    "input_bytes", "assets", "drawings", "instructions", "work", "simulation_ticks", "cache_bytes"};
static_assert(preparation_resource_names.size() == std::size_t(PreparationResource::Count));
struct PreparationBudget {
  std::uint64_t input_bytes{32 * 1024 * 1024}, assets{8}, drawings{32}, instructions{10000}, work{10000000},
      simulation_ticks{1000001}, cache_bytes{64 * 1024 * 1024};
  auto limits() const noexcept {
    return std::array{input_bytes, assets, drawings, instructions, work, simulation_ticks, cache_bytes};
  }
};
class PreparationError : public std::runtime_error {
public:
  PreparationError(PreparationFailure code, PreparationResource resource, std::string_view phase)
      : std::runtime_error(std::string(phase) + ": " +
                           (code == PreparationFailure::Cancelled ? "Preparation cancelled"
                            : code == PreparationFailure::DeadlineExceeded
                                ? "Preparation deadline exceeded"
                                : std::string("Preparation budget exceeded: ") +
                                      preparation_resource_names.at(std::size_t(resource)))),
        code_(code), resource_(resource), phase_(phase) {}
  PreparationFailure code() const noexcept { return code_; }
  PreparationResource resource() const noexcept { return resource_; }
  const std::string &phase() const noexcept { return phase_; }

private:
  PreparationFailure code_;
  PreparationResource resource_;
  std::string phase_;
};
/// One worker owns the counters. Other threads may only request_stop() on the
/// associated stop_source. Never use this context from the cyclic dispatcher.
class PreparationContext {
public:
  using Clock = std::chrono::steady_clock;
  explicit PreparationContext(PreparationBudget budget = {}, std::stop_token stop = {},
                              Clock::time_point deadline = Clock::time_point::max())
      : limits_(budget.limits()), stop_(stop), deadline_(deadline) {}
  PreparationContext(const PreparationContext &) = delete;
  PreparationContext &operator=(const PreparationContext &) = delete;
  void checkpoint(std::string_view phase) {
    if (failure_ != PreparationFailure::None)
      throw PreparationError(failure_, failed_resource_, failed_phase_);
    if (stop_.stop_requested())
      fail(PreparationFailure::Cancelled, PreparationResource::Count, phase);
    if (deadline_ != Clock::time_point::max() && Clock::now() >= deadline_)
      fail(PreparationFailure::DeadlineExceeded, PreparationResource::Count, phase);
  }
  void consume(PreparationResource resource, std::uint64_t amount, std::string_view phase) {
    checkpoint(phase);
    const auto i = std::size_t(resource);
    if (i >= used_.size())
      throw std::invalid_argument("Invalid preparation resource");
    if (amount > limits_[i] - used_[i])
      fail(PreparationFailure::BudgetExceeded, resource, phase);
    used_[i] += amount;
  }
  std::uint64_t used(PreparationResource resource) const { return used_.at(std::size_t(resource)); }
  PreparationFailure failure() const noexcept { return failure_; }

private:
  [[noreturn]] void fail(PreparationFailure code, PreparationResource resource, std::string_view phase) {
    failure_ = code;
    failed_resource_ = resource;
    failed_phase_ = phase;
    throw PreparationError(code, resource, phase);
  }
  std::array<std::uint64_t, 7> limits_, used_{};
  std::stop_token stop_;
  Clock::time_point deadline_;
  PreparationFailure failure_{PreparationFailure::None};
  PreparationResource failed_resource_{PreparationResource::Count};
  std::string failed_phase_;
};
inline void preparation_checkpoint(PreparationContext *context, std::string_view phase) {
  if (context)
    context->checkpoint(phase);
}
inline void preparation_consume(PreparationContext *context, PreparationResource resource,
                                std::uint64_t amount, std::string_view phase) {
  if (context)
    context->consume(resource, amount, phase);
}
} // namespace motion
