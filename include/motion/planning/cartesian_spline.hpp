#pragma once
#include "motion/kinematics/interval_kinematics.hpp"
#include "motion/planning/pose_path.hpp"
namespace motion {
struct CartesianSplineOptions {
  double position_tolerance_m{.0002}, orientation_tolerance_rad{.0005}, max_joint_step{.35};
  std::size_t initial_segments{16}, max_segments{256}, max_enclosure_depth{12}, max_enclosures{100000};
};
class CartesianSplinePath;
struct CartesianSplineResult {
  PlanStatus status{PlanStatus::InvalidInput};
  std::shared_ptr<const CartesianSplinePath> path;
  std::string diagnostic;
  explicit operator bool() const { return status == PlanStatus::Ready && bool(path); }
};
CartesianSplineResult map_cartesian_spline(std::shared_ptr<const IKinematics>,
                                           std::shared_ptr<const IPosePath>, const Vector &,
                                           CartesianSplineOptions = {});
class CartesianSplinePath final : public IBoundedJointPath {
public:
  JointPathBounds bounds() const override { return bounds_; }
  std::size_t segments() const { return controls_.size(); }
  const auto &kinematics() const { return model_; }
  double admitted_position_error_m() const { return position_error_; }
  double admitted_orientation_error_rad() const { return rotation_error_; }
  JointPathPoint evaluate(double s) const override {
    path_detail::parameter(s);
    const auto count = controls_.size(), k = std::min(count - 1, std::size_t(s * count));
    const long double u = s == 1 ? 1 : static_cast<long double>(s) * count - k;
    JointPathPoint r;
    const auto n = model_->dof();
    r.q.resize(n);
    r.q_s.resize(n);
    r.q_ss.resize(n);
    r.q_sss.resize(n);
    std::array<Vector *, 4> out{&r.q, &r.q_s, &r.q_ss, &r.q_sss};
    for (std::size_t i = 0; i < n; ++i) {
      auto b = controls_[k][i];
      for (int o = 0; o < 4; ++o) {
        (*out[o])[i] = double(polynomial_detail::value(b, 5 - o, u));
        if (o < 3)
          b = polynomial_detail::derivative(b, 5 - o, 1.L / count);
      }
    }
    r.tcp.pose = model_->fk(r.q);
    KinematicDerivatives d;
    if (const auto *diff = dynamic_cast<const IDifferentialKinematics *>(model_.get()))
      d = diff->differentiate(r.q, r.q_s, r.q_ss, r.q_sss);
    else
      for (int i = 0; i < 3; ++i) {
        d.velocity.data[i] = r.q_s[i];
        d.acceleration.data[i] = r.q_ss[i];
        d.jerk.data[i] = r.q_sss[i];
      }
    const std::array<Twist, 3> values{d.velocity, d.acceleration, d.jerk};
    for (int o = 0; o < 3; ++o)
      for (int i = 0; i < 3; ++i) {
        r.tcp.derivatives[o][i] = values[o].data[i];
        r.tcp.angular_derivatives[o][i] = values[o].data[i + 3];
      }
    return r;
  }
  std::vector<enclosure::Jet> enclose(double lo, double hi) const {
    path_detail::parameter(lo);
    path_detail::parameter(hi);
    if (lo > hi)
      throw std::invalid_argument("Reversed spline interval");
    std::vector<enclosure::Jet> out(model_->dof());
    bool first = true;
    const auto count = controls_.size();
    const auto begin = std::min(count - 1, std::size_t(lo * count)),
               end = std::min(count - 1, std::size_t(hi * count));
    for (auto k = begin; k <= end; ++k) {
      auto q = enclosure::polynomial_jets(
          controls_[k], 1.L / count, double(std::clamp(static_cast<long double>(lo) * count - k, 0.L, 1.L)),
          double(std::clamp(static_cast<long double>(hi) * count - k, 0.L, 1.L)));
      for (std::size_t i = 0; i < q.size(); ++i)
        for (int o = 0; o < 4; ++o)
          out[i].d[o] = first ? q[i].d[o]
                              : enclosure::Interval(std::min(out[i].d[o].lo, q[i].d[o].lo),
                                                    std::max(out[i].d[o].hi, q[i].d[o].hi));
      first = false;
    }
    return out;
  }

private:
  friend CartesianSplineResult map_cartesian_spline(std::shared_ptr<const IKinematics>,
                                                    std::shared_ptr<const IPosePath>, const Vector &,
                                                    CartesianSplineOptions);
  std::shared_ptr<const IKinematics> model_;
  std::vector<std::vector<polynomial_detail::Control>> controls_;
  JointPathBounds bounds_;
  double position_error_{}, rotation_error_{};
};
inline CartesianSplineResult map_cartesian_spline(std::shared_ptr<const IKinematics> model,
                                                  std::shared_ptr<const IPosePath> desired,
                                                  const Vector &seed, CartesianSplineOptions options) {
  const auto fail = [](PlanStatus s, const char *d) { return CartesianSplineResult{s, {}, d}; };
  if (!model || !desired || desired->pieces() == 0 || desired->pieces() > 256 ||
      !kinematic_detail::finite(seed, model->dof()) || options.initial_segments < 2 ||
      options.max_segments < options.initial_segments || options.max_segments > 2048 ||
      options.max_enclosure_depth > 20 || !options.max_enclosures)
    return fail(PlanStatus::InvalidInput, "Invalid spline model/budget");
  for (double x : {options.position_tolerance_m, options.orientation_tolerance_rad, options.max_joint_step})
    if (!std::isfinite(x) || x <= 0)
      return fail(PlanStatus::InvalidInput, "Invalid spline tolerances");
  try {
    const auto first = model->fk(seed), requested = desired->evaluate(0);
    if (kinematic_detail::norm(kinematic_detail::sub(first.position, requested.position)) >
            options.position_tolerance_m ||
        rotation_distance(first.orientation, requested.orientation) > options.orientation_tolerance_rad)
      return fail(PlanStatus::InvalidInput, "Spline initial pose mismatch");
    const auto *diff = dynamic_cast<const IDifferentialKinematics *>(model.get());
    KinematicJointLimits limits{Vector(seed.size(), -1e6), Vector(seed.size(), 1e6)};
    if (diff)
      limits = diff->position_limits();
    else if (!dynamic_cast<const CartesianGantry *>(model.get()))
      return fail(PlanStatus::Unsupported, "No spline enclosure provider");
    for (std::size_t count =
             ((options.initial_segments + desired->pieces() - 1) / desired->pieces()) * desired->pieces();
         count <= options.max_segments; count *= 2) {
      const auto n = seed.size();
      const long double h = 1.L / count;
      std::vector<Vector> q{seed};
      bool okay = true;
      for (std::size_t k = 1; k <= count; ++k) {
        auto next = model->ik(desired->evaluate(double(k) / count), q.back());
        if (!next) {
          okay = false;
          break;
        }
        for (std::size_t i = 0; i < n; ++i)
          if (std::abs((*next)[i] - q.back()[i]) > options.max_joint_step)
            okay = false;
        if (!okay)
          break;
        q.push_back(*next);
      }
      if (!okay)
        continue;
      std::vector<Vector> v(count + 1, Vector(n)), a = v;
      for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t k = 1; k < count; ++k) {
          v[k][i] = double((q[k + 1][i] - q[k - 1][i]) / (2 * h));
          a[k][i] = double((q[k + 1][i] - 2 * q[k][i] + q[k - 1][i]) / (h * h));
        }
        v[0][i] = double((-3 * q[0][i] + 4 * q[1][i] - q[2][i]) / (2 * h));
        v[count][i] = double((3 * q[count][i] - 4 * q[count - 1][i] + q[count - 2][i]) / (2 * h));
        a[0][i] = a[1][i];
        a[count][i] = a[count - 1][i];
      }
      auto candidate = std::shared_ptr<CartesianSplinePath>(new CartesianSplinePath);
      candidate->model_ = model;
      candidate->bounds_ = {seed, seed, {Vector(n), Vector(n), Vector(n)}, {}, {}};
      for (std::size_t k = 0; k < count; ++k) {
        std::vector<polynomial_detail::Control> b;
        for (std::size_t i = 0; i < n; ++i)
          b.push_back(polynomial_detail::hermite(q[k][i], v[k][i], a[k][i], q[k + 1][i], v[k + 1][i],
                                                 a[k + 1][i], h));
        candidate->controls_.push_back(std::move(b));
      }
      std::size_t calls = 0;
      const auto admit = [&](auto &&self, const std::vector<polynomial_detail::Control> &b, double begin,
                             double span, std::size_t depth) -> bool {
        if (++calls > options.max_enclosures)
          return false;
        bool accepted = false;
        try {
          const auto jets = enclosure::polynomial_jets(b, span);
          const auto image = enclosure::evaluate(*model, jets);
          accepted = true;
          for (std::size_t i = 0; i < n; ++i)
            if (jets[i].d[0].lo < limits.lower[i] || jets[i].d[0].hi > limits.upper[i])
              accepted = false;
          std::array<double, 3> linear{}, angular{};
          for (int o = 0; o < 3; ++o) {
            PathVector3 l{}, a{};
            for (int i = 0; i < 3; ++i) {
              l[i] = image.position[i].d[o + 1].absolute();
              a[i] = image.angular[o][i].absolute();
            }
            linear[o] = kinematic_detail::norm(l);
            angular[o] = kinematic_detail::norm(a);
          }
          const double mid = begin + span / 2;
          const auto actual = candidate->evaluate(mid).tcp.pose, goal = desired->evaluate(mid);
          const double p = kinematic_detail::norm(kinematic_detail::sub(actual.position, goal.position)) +
                           (linear[0] + desired->linear_speed_bound()) * span / 2;
          const double r = rotation_distance(actual.orientation, goal.orientation) +
                           (angular[0] + desired->angular_speed_bound()) * span / 2;
          accepted = accepted && p <= options.position_tolerance_m && r <= options.orientation_tolerance_rad;
          if (accepted) {
            candidate->position_error_ = std::max(candidate->position_error_, p);
            candidate->rotation_error_ = std::max(candidate->rotation_error_, r);
            for (std::size_t i = 0; i < n; ++i) {
              candidate->bounds_.lower[i] = std::min(candidate->bounds_.lower[i], jets[i].d[0].lo);
              candidate->bounds_.upper[i] = std::max(candidate->bounds_.upper[i], jets[i].d[0].hi);
              for (int o = 0; o < 3; ++o)
                candidate->bounds_.joint_abs[o][i] =
                    std::max(candidate->bounds_.joint_abs[o][i], jets[i].d[o + 1].absolute());
            }
            for (int o = 0; o < 3; ++o) {
              candidate->bounds_.linear_abs[o] = std::max(candidate->bounds_.linear_abs[o], linear[o]);
              candidate->bounds_.angular_abs[o] = std::max(candidate->bounds_.angular_abs[o], angular[o]);
            }
            return true;
          }
        } catch (const std::domain_error &) {
        }
        if (depth >= options.max_enclosure_depth)
          return false;
        std::vector<polynomial_detail::Control> left, right;
        for (const auto &axis : b) {
          auto halves = polynomial_detail::split(axis);
          left.push_back(halves.first);
          right.push_back(halves.second);
        }
        return self(self, left, begin, span / 2, depth + 1) &&
               self(self, right, begin + span / 2, span / 2, depth + 1);
      };
      for (std::size_t k = 0; k < count; ++k)
        if (!admit(admit, candidate->controls_[k], double(k) / count, double(h), 0)) {
          okay = false;
          break;
        }
      if (okay)
        return {PlanStatus::Ready, std::move(candidate),
                "Bounded numerical pose tube and differential envelopes"};
    }
    return fail(PlanStatus::Infeasible, "Spline IK/tube admission failed within budget");
  } catch (const std::exception &e) {
    return {PlanStatus::Infeasible, {}, e.what()};
  }
}
} // namespace motion
