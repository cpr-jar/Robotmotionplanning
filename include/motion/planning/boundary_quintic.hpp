#pragma once
#include "motion/math/bernstein_polynomial.hpp"
#include "motion/planning/joint_trajectory.hpp"
namespace motion {
struct JointPolynomialBounds {
  Vector lower, upper;
  std::array<Vector, 3> absolute;
};
struct BoundaryPolynomialOptions {
  std::size_t max_depth{16}, max_leaves{100000};
};
class BoundaryQuinticTrajectory;
struct BoundaryPolynomialResult {
  PlanStatus status{PlanStatus::InvalidInput};
  std::shared_ptr<const BoundaryQuinticTrajectory> trajectory;
  std::string diagnostic;
  explicit operator bool() const { return status == PlanStatus::Ready && bool(trajectory); }
};
BoundaryPolynomialResult plan_boundary_quintic(const JointLimits &, const JointState &, const JointTarget &,
                                               double, BoundaryPolynomialOptions = {});
class BoundaryQuinticTrajectory final : public ITrajectory {
public:
  double duration() const noexcept override { return duration_; }
  const JointState &initial_state() const { return initial_; }
  const JointState &terminal_state() const { return terminal_; }
  const JointLimits &joint_limits() const { return limits_; }
  const JointPolynomialBounds &bounds() const { return bounds_; }
  const auto &control_points() const { return controls_; }
  JointState sample(double t) const override { return sample_with_jerk(t).first; }
  std::pair<JointState, Vector> sample_with_jerk(double t) const {
    if (!std::isfinite(t) || t < 0 || t > duration_)
      throw std::invalid_argument("Polynomial time outside interval");
    auto r = initial_;
    Vector j(r.q.size());
    const long double u = static_cast<long double>(t) / duration_;
    for (std::size_t i = 0; i < r.q.size(); ++i) {
      auto b = controls_[i];
      r.q[i] = double(polynomial_detail::value(b, 5, u));
      b = polynomial_detail::derivative(b, 5, duration_);
      r.dq[i] = double(polynomial_detail::value(b, 4, u));
      b = polynomial_detail::derivative(b, 4, duration_);
      r.ddq[i] = double(polynomial_detail::value(b, 3, u));
      b = polynomial_detail::derivative(b, 3, duration_);
      j[i] = double(polynomial_detail::value(b, 2, u));
    }
    r.timestamp_s = initial_.timestamp_s + t;
    if (t == 0)
      r = initial_;
    if (t == duration_)
      r = terminal_;
    return {std::move(r), std::move(j)};
  }

private:
  friend BoundaryPolynomialResult plan_boundary_quintic(const JointLimits &, const JointState &,
                                                        const JointTarget &, double,
                                                        BoundaryPolynomialOptions);
  BoundaryQuinticTrajectory() = default;
  JointLimits limits_;
  JointState initial_, terminal_;
  double duration_{};
  std::vector<polynomial_detail::Control> controls_;
  JointPolynomialBounds bounds_;
};
inline BoundaryPolynomialResult plan_boundary_quintic(const JointLimits &l, const JointState &s,
                                                      const JointTarget &g, double T,
                                                      BoundaryPolynomialOptions opts) {
  const auto fail = [](PlanStatus status, const char *d) { return BoundaryPolynomialResult{status, {}, d}; };
  const auto n = l.min_position.size();
  try {
    check_limits(l, n);
    check_state(s, n);
  } catch (...) {
    return fail(PlanStatus::InvalidInput, "Invalid polynomial limits/state");
  }
  const auto finite = [&](const Vector &v) {
    return v.size() == n && std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); });
  };
  if (!finite(g.q) || !finite(g.dq) || !finite(g.ddq) || !std::isfinite(s.timestamp_s) || !std::isfinite(T) ||
      T <= 0 || opts.max_depth > 24 || !opts.max_leaves)
    return fail(PlanStatus::InvalidInput, "Invalid polynomial endpoints/duration");
  const double end = s.timestamp_s + T, mag = std::max(std::abs(s.timestamp_s), std::abs(end));
  if (!std::isfinite(end) || T <= 4 * (std::nextafter(mag, INFINITY) - mag))
    return fail(PlanStatus::Infeasible, "Insufficient polynomial clock resolution");
  JointState terminal{g.q, g.dq, g.ddq, end};
  if (!within_joint_limits(s, l, 0) || !within_joint_limits(terminal, l, 0))
    return fail(PlanStatus::Infeasible, "Polynomial boundary outside limits");
  auto candidate = std::shared_ptr<BoundaryQuinticTrajectory>(new BoundaryQuinticTrajectory);
  candidate->limits_ = l;
  candidate->initial_ = s;
  candidate->terminal_ = terminal;
  candidate->duration_ = T;
  candidate->bounds_ = {Vector(n, INFINITY), Vector(n, -INFINITY), {Vector(n), Vector(n), Vector(n)}};
  std::size_t leaves = 0;
  for (std::size_t axis = 0; axis < n; ++axis) {
    const auto original =
        polynomial_detail::hermite(s.q[axis], s.dq[axis], s.ddq[axis], g.q[axis], g.dq[axis], g.ddq[axis], T);
    candidate->controls_.push_back(original);
    const auto admit = [&](auto &&self, polynomial_detail::Control b, long double span,
                           std::size_t depth) -> bool {
      if (++leaves > opts.max_leaves)
        return false;
      const auto mm = std::minmax_element(b.begin(), b.end());
      bool ok = *mm.first >= l.min_position[axis] && *mm.second <= l.max_position[axis];
      std::array<double, 3> peak{};
      auto derivative = b;
      const double caps[3] = {l.max_velocity[axis], l.max_acceleration[axis], l.max_jerk[axis]};
      for (int order = 0; order < 3; ++order) {
        derivative = polynomial_detail::derivative(derivative, 5 - order, span);
        for (int i = 0; i < 5 - order; ++i)
          peak[order] = std::max(peak[order], double(std::abs(derivative[i])));
        if (peak[order] > caps[order] * (1 + 128 * std::numeric_limits<double>::epsilon()))
          ok = false;
      }
      if (ok) {
        candidate->bounds_.lower[axis] = std::min(candidate->bounds_.lower[axis], double(*mm.first));
        candidate->bounds_.upper[axis] = std::max(candidate->bounds_.upper[axis], double(*mm.second));
        for (int i = 0; i < 3; ++i)
          candidate->bounds_.absolute[i][axis] = std::max(candidate->bounds_.absolute[i][axis], peak[i]);
        return true;
      }
      if (depth >= opts.max_depth)
        return false;
      auto halves = polynomial_detail::split(b);
      return self(self, halves.first, span / 2, depth + 1) && self(self, halves.second, span / 2, depth + 1);
    };
    if (!admit(admit, original, T, 0))
      return fail(PlanStatus::Infeasible, "Polynomial envelope rejected or budget exhausted");
  }
  return {PlanStatus::Ready, std::move(candidate), "Full-interval Bernstein q/v/a/j admission (numerical)"};
}
} // namespace motion
