#pragma once
#include "cartesian_spline.hpp"
#include "path_timing.hpp"
namespace motion {
class IEnvelopeConstraint {
public:
  virtual ~IEnvelopeConstraint() = default;
  virtual bool admit(const IKinematics &, const std::vector<enclosure::Jet> &,
                     const enclosure::Result &) const = 0;
};
struct EnvelopeCheckOptions {
  std::size_t max_depth{16}, max_intervals{100000};
};
struct EnvelopeCheckResult {
  bool admitted{};
  std::size_t intervals{};
  std::array<double, 3> linear{}, angular{};
  std::string diagnostic;
  explicit operator bool() const { return admitted; }
};
namespace constraint_detail {
template <class State>
EnvelopeCheckResult check(const IKinematics &model, double T, const CartesianLimits &caps,
                          const std::vector<std::shared_ptr<const IEnvelopeConstraint>> &constraints,
                          EnvelopeCheckOptions options, State state) {
  EnvelopeCheckResult r;
  for (double x : {caps.max_linear_velocity, caps.max_linear_acceleration, caps.max_linear_jerk,
                   caps.max_angular_velocity, caps.max_angular_acceleration, caps.max_angular_jerk})
    if (!std::isfinite(x) || x <= 0) {
      r.diagnostic = "Invalid TCP caps";
      return r;
    }
  if (!std::isfinite(T) || T <= 0 || options.max_depth > 24 || !options.max_intervals) {
    r.diagnostic = "Invalid envelope budget";
    return r;
  }
  for (const auto &c : constraints)
    if (!c) {
      r.diagnostic = "Missing constraint";
      return r;
    }
  const auto recurse = [&](auto &&self, double lo, double hi, std::size_t depth) -> bool {
    if (++r.intervals > options.max_intervals)
      return false;
    try {
      const auto q = state(lo, hi);
      const auto image = enclosure::evaluate(model, q);
      bool ok = true;
      std::array<double, 3> linear{}, angular{};
      const double lc[3] = {caps.max_linear_velocity, caps.max_linear_acceleration, caps.max_linear_jerk},
                   ac[3] = {caps.max_angular_velocity, caps.max_angular_acceleration, caps.max_angular_jerk};
      for (int o = 0; o < 3; ++o) {
        PathVector3 l{}, a{};
        for (int i = 0; i < 3; ++i) {
          l[i] = image.position[i].d[o + 1].absolute();
          a[i] = image.angular[o][i].absolute();
        }
        linear[o] = kinematic_detail::norm(l);
        angular[o] = kinematic_detail::norm(a);
        if (linear[o] > lc[o] || angular[o] > ac[o])
          ok = false;
      }
      for (const auto &c : constraints)
        if (ok && !c->admit(model, q, image))
          ok = false;
      if (ok) {
        for (int i = 0; i < 3; ++i) {
          r.linear[i] = std::max(r.linear[i], linear[i]);
          r.angular[i] = std::max(r.angular[i], angular[i]);
        }
        return true;
      }
    } catch (const std::domain_error &) {
    }
    if (depth >= options.max_depth)
      return false;
    const double mid = lo + (hi - lo) / 2;
    if (mid == lo || mid == hi)
      return false;
    return self(self, lo, mid, depth + 1) && self(self, mid, hi, depth + 1);
  };
  r.admitted = recurse(recurse, 0, T, 0);
  r.diagnostic = r.admitted ? "Whole-interval constraints admitted" : "Envelope rejected or budget exhausted";
  return r;
}
} // namespace constraint_detail
inline EnvelopeCheckResult check_polynomial_constraints(
    const IKinematics &model, const BoundaryQuinticTrajectory &t, const CartesianLimits &caps,
    const std::vector<std::shared_ptr<const IEnvelopeConstraint>> &c = {}, EnvelopeCheckOptions o = {}) {
  return constraint_detail::check(model, t.duration(), caps, c, o, [&](double lo, double hi) {
    return enclosure::polynomial_jets(t.control_points(), t.duration(), lo / t.duration(), hi / t.duration());
  });
}
struct WorkspaceBox {
  std::array<double, 3> lower{}, upper{};
};
class TcpObstacleConstraint final : public IEnvelopeConstraint {
public:
  TcpObstacleConstraint(std::vector<WorkspaceBox> boxes, double radius)
      : boxes_(std::move(boxes)), radius_(radius) {
    if (!std::isfinite(radius) || radius < 0)
      throw std::invalid_argument("Invalid obstacle radius");
    for (auto b : boxes_)
      for (int i = 0; i < 3; ++i)
        if (!std::isfinite(b.lower[i]) || !std::isfinite(b.upper[i]) || b.lower[i] > b.upper[i])
          throw std::invalid_argument("Invalid obstacle box");
  }
  bool admit(const IKinematics &, const std::vector<enclosure::Jet> &,
             const enclosure::Result &r) const override {
    for (const auto &b : boxes_) {
      bool separate = false;
      for (int i = 0; i < 3; ++i)
        if (r.position[i].d[0].hi + radius_ < b.lower[i] || r.position[i].d[0].lo - radius_ > b.upper[i])
          separate = true;
      if (!separate)
        return false;
    }
    return true;
  }

private:
  std::vector<WorkspaceBox> boxes_;
  double radius_;
};
} // namespace motion
#ifdef MOTION_WITH_RUCKIG
#include "joint_stop.hpp"
namespace motion {
inline EnvelopeCheckResult
check_stop_constraints(const IKinematics &model, const JointStopTrajectory &stop, const CartesianLimits &caps,
                       const std::vector<std::shared_ptr<const IEnvelopeConstraint>> &constraints = {},
                       EnvelopeCheckOptions options = {}) {
  return constraint_detail::check(
      model, std::max(.001, stop.duration()), caps, constraints, options, [&](double lo, double hi) {
        const auto b = stop.enclose_state(std::min(lo, stop.duration()), std::min(hi, stop.duration()));
        std::vector<enclosure::Jet> q(b.size());
        for (std::size_t i = 0; i < b.size(); ++i)
          for (int o = 0; o < 4; ++o)
            q[i].d[o] = enclosure::Interval(b[i][o].first, b[i][o].second);
        return q;
      });
}
} // namespace motion
#endif
