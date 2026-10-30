#pragma once
#include "motion/kinematics/differential_kinematics.hpp"
#include "motion/math/derivative_jets.hpp"
namespace motion {
namespace reference_detail {
inline void require_q(const Vector &q, const KinematicJointLimits &l) {
  if (!kinematic_detail::finite(q, l.lower.size()))
    throw std::invalid_argument("Reference joint dimension");
  for (std::size_t i = 0; i < q.size(); ++i)
    if (q[i] < l.lower[i] || q[i] > l.upper[i])
      throw std::invalid_argument("Reference joint range");
}
inline void limits(const KinematicJointLimits &l, std::size_t n) {
  if (!kinematic_detail::finite(l.lower, n) || !kinematic_detail::finite(l.upper, n))
    throw std::invalid_argument("Reference limit dimension");
  for (std::size_t i = 0; i < n; ++i)
    if (l.lower[i] >= l.upper[i])
      throw std::invalid_argument("Reference limit range");
}
inline std::optional<double> lift(double angle, double seed, double lo, double hi) {
  const double tau = 2 * kinematic_detail::pi;
  const double minimum = std::ceil((lo - angle) / tau), maximum = std::floor((hi - angle) / tau);
  if (minimum > maximum)
    return {};
  const double q = angle + tau * std::clamp(std::round((seed - angle) / tau), minimum, maximum);
  if (std::abs(q - seed) > kinematic_detail::pi)
    return {};
  return q;
}
inline kinematic_detail::V3 solve(const std::array<kinematic_detail::V3, 3> &A, kinematic_detail::V3 b) {
  using namespace kinematic_detail;
  const auto x = cross(A[1], A[2]), y = cross(A[2], A[0]), z = cross(A[0], A[1]);
  const double d = dot(A[0], x);
  if (!std::isfinite(d) || std::abs(d) < 1e-12)
    throw std::domain_error("Singular Delta constraints");
  return scale(add(add(scale(x, b[0]), scale(y, b[1])), scale(z, b[2])), 1 / d);
}
} // namespace reference_detail
struct ScaraGeometry {
  double first_link_m{}, second_link_m{};
};
class ScaraKinematics final : public IDifferentialKinematics {
public:
  ScaraKinematics(ScaraGeometry g, KinematicJointLimits l) : geometry_(g), limits_(std::move(l)) {
    reference_detail::limits(limits_, 4);
    for (double x : {g.first_link_m, g.second_link_m})
      if (!std::isfinite(x) || x <= 0)
        throw std::invalid_argument("Invalid SCARA geometry");
  }
  std::size_t dof() const override { return 4; }
  const KinematicJointLimits &position_limits() const noexcept override { return limits_; }
  const ScaraGeometry &geometry() const { return geometry_; }
  Pose fk(const Vector &q) const override {
    reference_detail::require_q(q, limits_);
    const double yaw = q[0] + q[1] + q[3];
    return {{geometry_.first_link_m * std::cos(q[0]) + geometry_.second_link_m * std::cos(q[0] + q[1]),
             geometry_.first_link_m * std::sin(q[0]) + geometry_.second_link_m * std::sin(q[0] + q[1]), q[2]},
            {std::cos(yaw / 2), 0, 0, std::sin(yaw / 2)}};
  }
  std::optional<Vector> ik(const Pose &p, const Vector &seed) const override {
    if (!kinematic_detail::finite_pose(p) || !kinematic_detail::finite(seed, 4) ||
        std::hypot(p.orientation[1], p.orientation[2]) > 1e-9 || p.position[2] < limits_.lower[2] ||
        p.position[2] > limits_.upper[2])
      return {};
    const double l1 = geometry_.first_link_m, l2 = geometry_.second_link_m,
                 c = (p.position[0] * p.position[0] + p.position[1] * p.position[1] - l1 * l1 - l2 * l2) /
                     (2 * l1 * l2);
    if (c < -1 || c > 1)
      return {};
    std::optional<Vector> best;
    double cost = INFINITY;
    for (double q2 : {std::acos(c), -std::acos(c)}) {
      const double q1 =
          std::atan2(p.position[1], p.position[0]) - std::atan2(l2 * std::sin(q2), l1 + l2 * std::cos(q2));
      const double yaw = 2 * std::atan2(p.orientation[3], p.orientation[0]);
      Vector raw{q1, q2, p.position[2], yaw - q1 - q2}, q(4);
      bool good = true;
      for (int i = 0; i < 4; ++i) {
        if (i == 2) {
          q[i] = raw[i];
          continue;
        }
        auto x = reference_detail::lift(raw[i], seed[i], limits_.lower[i], limits_.upper[i]);
        if (!x) {
          good = false;
          break;
        }
        q[i] = *x;
      }
      if (good) {
        double distance = 0;
        for (int i = 0; i < 4; ++i)
          distance += (q[i] - seed[i]) * (q[i] - seed[i]);
        if (distance < cost) {
          best = q;
          cost = distance;
        }
      }
    }
    return best;
  }
  KinematicDerivatives differentiate(const Vector &q, const Vector &v, const Vector &a,
                                     const Vector &j) const override {
    reference_detail::require_q(q, limits_);
    for (const auto *x : {&v, &a, &j})
      if (!kinematic_detail::finite(*x, 4))
        throw std::invalid_argument("SCARA derivative dimension");
    using namespace differential_math;
    Jet<double> q1({q[0], v[0], a[0], j[0]}), q2({q[1], v[1], a[1], j[1]});
    const auto x = Jet<double>(geometry_.first_link_m) * cosine(q1) +
                   Jet<double>(geometry_.second_link_m) * cosine(q1 + q2),
               y = Jet<double>(geometry_.first_link_m) * sine(q1) +
                   Jet<double>(geometry_.second_link_m) * sine(q1 + q2);
    KinematicDerivatives out;
    std::array<Twist *, 3> result{&out.velocity, &out.acceleration, &out.jerk};
    const std::array<const Vector *, 3> values{&v, &a, &j};
    for (int o = 0; o < 3; ++o) {
      result[o]->data[0] = x.d[o + 1];
      result[o]->data[1] = y.d[o + 1];
      result[o]->data[2] = (*values[o])[2];
      result[o]->data[5] = (*values[o])[0] + (*values[o])[1] + (*values[o])[3];
    }
    return out;
  }
  std::vector<Vector> jacobian(const Vector &q) const override {
    std::vector<Vector> J(6, Vector(4));
    for (int i = 0; i < 4; ++i) {
      Vector v(4);
      v[i] = 1;
      const auto column = differentiate(q, v, Vector(4), Vector(4)).velocity;
      for (int r = 0; r < 6; ++r)
        J[r][i] = column.data[r];
    }
    return J;
  }
  SingularityInfo singularity(const Vector &q) const override {
    reference_detail::require_q(q, limits_);
    const double m = std::abs(std::sin(q[1]));
    return {m, m, m < 1e-6};
  }

private:
  ScaraGeometry geometry_;
  KinematicJointLimits limits_;
};
struct DeltaGeometry {
  double base_radius_m{}, tool_radius_m{}, upper_arm_m{}, lower_arm_m{};
};
class DeltaKinematics final : public IDifferentialKinematics {
  using V3 = kinematic_detail::V3;

public:
  DeltaKinematics(DeltaGeometry g, KinematicJointLimits l) : geometry_(g), limits_(std::move(l)) {
    reference_detail::limits(limits_, 3);
    for (double x : {g.base_radius_m, g.upper_arm_m, g.lower_arm_m})
      if (!std::isfinite(x) || x <= 0)
        throw std::invalid_argument("Invalid Delta geometry");
    if (!std::isfinite(g.tool_radius_m) || g.tool_radius_m < 0 || g.base_radius_m <= g.tool_radius_m)
      throw std::invalid_argument("Invalid Delta radii");
  }
  std::size_t dof() const override { return 3; }
  const auto &geometry() const { return geometry_; }
  const KinematicJointLimits &position_limits() const noexcept override { return limits_; }
  Pose fk(const Vector &q) const override {
    using namespace kinematic_detail;
    reference_detail::require_q(q, limits_);
    const auto c = centers(q);
    const auto ab = sub(c[1], c[0]);
    const double d = norm(ab);
    if (d < 1e-12)
      throw std::domain_error("Degenerate Delta centers");
    const auto ex = scale(ab, 1 / d), ac = sub(c[2], c[0]);
    const double i = dot(ex, ac);
    const auto perpendicular = sub(ac, scale(ex, i));
    const double j = norm(perpendicular);
    if (j < 1e-12)
      throw std::domain_error("Degenerate Delta plane");
    const auto ey = scale(perpendicular, 1 / j), ez = cross(ex, ey);
    const double x = d / 2, y = (i * i + j * j - 2 * i * x) / (2 * j),
                 height2 = geometry_.lower_arm_m * geometry_.lower_arm_m - x * x - y * y;
    if (height2 <= 1e-14)
      throw std::domain_error("Delta outside lower-assembly workspace");
    const auto middle = add(c[0], add(scale(ex, x), scale(ey, y)));
    auto p1 = add(middle, scale(ez, std::sqrt(height2))), p2 = sub(middle, scale(ez, std::sqrt(height2)));
    return {p1[2] < p2[2] ? p1 : p2, {1, 0, 0, 0}};
  }
  std::optional<Vector> ik(const Pose &pose, const Vector &seed) const override {
    using namespace kinematic_detail;
    if (!fixed_orientation(pose) || !finite(seed, 3))
      return {};
    Vector q(3);
    const double R = geometry_.base_radius_m - geometry_.tool_radius_m, u = geometry_.upper_arm_m,
                 l = geometry_.lower_arm_m;
    for (int i = 0; i < 3; ++i) {
      const auto r = radial(i);
      const double A = dot(pose.position, r) - R, B = -pose.position[2];
      const auto p = sub(pose.position, scale(r, R));
      const double C = (dot(p, p) + u * u - l * l) / (2 * u), n = std::hypot(A, B);
      if (n <= 0 || std::abs(C) > n)
        return {};
      const double phi = std::atan2(B, A), theta = std::acos(std::clamp(C / n, -1., 1.));
      std::optional<double> best;
      for (double angle : {phi + theta, phi - theta}) {
        auto lifted = reference_detail::lift(angle, seed[i], limits_.lower[i], limits_.upper[i]);
        if (lifted && (!best || std::abs(*lifted - seed[i]) < std::abs(*best - seed[i])))
          best = lifted;
      }
      if (!best)
        return {};
      q[i] = *best;
    }
    try {
      if (norm(sub(fk(q).position, pose.position)) > 1e-7)
        return {};
    } catch (...) {
      return {};
    }
    return q;
  }
  KinematicDerivatives differentiate(const Vector &q, const Vector &v, const Vector &a,
                                     const Vector &j) const override {
    using namespace kinematic_detail;
    reference_detail::require_q(q, limits_);
    for (const auto *x : {&v, &a, &j})
      if (!finite(*x, 3))
        throw std::invalid_argument("Delta derivative dimension");
    const auto p = fk(q).position;
    const auto c = centers(q);
    std::array<V3, 3> rows, ev, ea, ej;
    V3 rhs{};
    for (int i = 0; i < 3; ++i) {
      const auto direction = radial(i);
      const auto eq = add(scale(direction, -geometry_.upper_arm_m * std::sin(q[i])),
                          V3{0, 0, -geometry_.upper_arm_m * std::cos(q[i])}),
                 eqq = add(scale(direction, -geometry_.upper_arm_m * std::cos(q[i])),
                           V3{0, 0, geometry_.upper_arm_m * std::sin(q[i])}),
                 eqqq = scale(eq, -1);
      rows[i] = sub(p, c[i]);
      ev[i] = scale(eq, v[i]);
      ea[i] = add(scale(eqq, v[i] * v[i]), scale(eq, a[i]));
      ej[i] = add(add(scale(eqqq, v[i] * v[i] * v[i]), scale(eqq, 3 * v[i] * a[i])), scale(eq, j[i]));
      rhs[i] = dot(rows[i], ev[i]);
    }
    const auto velocity = reference_detail::solve(rows, rhs);
    for (int i = 0; i < 3; ++i) {
      const auto d = sub(velocity, ev[i]);
      rhs[i] = dot(rows[i], ea[i]) - dot(d, d);
    }
    const auto acceleration = reference_detail::solve(rows, rhs);
    for (int i = 0; i < 3; ++i)
      rhs[i] = dot(rows[i], ej[i]) - 3 * dot(sub(velocity, ev[i]), sub(acceleration, ea[i]));
    const auto jerk = reference_detail::solve(rows, rhs);
    KinematicDerivatives out;
    for (int i = 0; i < 3; ++i) {
      out.velocity.data[i] = velocity[i];
      out.acceleration.data[i] = acceleration[i];
      out.jerk.data[i] = jerk[i];
    }
    require_finite(out);
    return out;
  }
  std::vector<Vector> jacobian(const Vector &q) const override {
    std::vector<Vector> J(6, Vector(3));
    for (int i = 0; i < 3; ++i) {
      Vector v(3);
      v[i] = 1;
      const auto column = differentiate(q, v, Vector(3), Vector(3)).velocity;
      for (int r = 0; r < 6; ++r)
        J[r][i] = column.data[r];
    }
    return J;
  }
  SingularityInfo singularity(const Vector &q) const override {
    using namespace kinematic_detail;
    try {
      const auto p = fk(q).position;
      const auto c = centers(q);
      std::array<V3, 3> rows;
      double transmission = 1;
      for (int i = 0; i < 3; ++i) {
        rows[i] = sub(p, c[i]);
        const auto eq = add(scale(radial(i), -geometry_.upper_arm_m * std::sin(q[i])),
                            V3{0, 0, -geometry_.upper_arm_m * std::cos(q[i])});
        transmission = std::min(transmission,
                                std::abs(dot(rows[i], eq)) / (geometry_.upper_arm_m * geometry_.lower_arm_m));
      }
      const double m = std::abs(dot(rows[0], cross(rows[1], rows[2]))) / std::pow(geometry_.lower_arm_m, 3);
      return {m, transmission, m < 1e-6 || transmission < 1e-6};
    } catch (...) {
      return {0, 0, true};
    }
  }

private:
  V3 radial(int i) const {
    const double angle = 2 * kinematic_detail::pi * i / 3;
    return {std::cos(angle), std::sin(angle), 0};
  }
  std::array<V3, 3> centers(const Vector &q) const {
    std::array<V3, 3> c;
    for (int i = 0; i < 3; ++i) {
      c[i] = kinematic_detail::scale(radial(i), geometry_.base_radius_m - geometry_.tool_radius_m +
                                                    geometry_.upper_arm_m * std::cos(q[i]));
      c[i][2] = -geometry_.upper_arm_m * std::sin(q[i]);
    }
    return c;
  }
  DeltaGeometry geometry_;
  KinematicJointLimits limits_;
};
} // namespace motion
