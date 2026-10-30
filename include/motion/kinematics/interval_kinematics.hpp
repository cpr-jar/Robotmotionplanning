#pragma once
#include "motion/kinematics/reference_kinematics.hpp"
#include "motion/kinematics/serial_chain_kinematics.hpp"
#include "motion/math/bernstein_polynomial.hpp"
#include "motion/math/derivative_jets.hpp"
namespace motion::enclosure {
inline double down(double x) {
  return std::nextafter(x, -INFINITY);
}
inline double up(double x) {
  return std::nextafter(x, INFINITY);
}
struct Interval {
  double lo{}, hi{};
  Interval() = default;
  Interval(double x) : lo(x), hi(x) {
    if (!std::isfinite(x))
      throw std::domain_error("Nonfinite interval");
  }
  Interval(double a, double b) : lo(a), hi(b) {
    if (!std::isfinite(a) || !std::isfinite(b) || a > b)
      throw std::domain_error("Invalid interval");
  }
  double absolute() const { return std::max(std::abs(lo), std::abs(hi)); }
};
inline Interval operator+(Interval a, Interval b) {
  // These identities are exact, including for floating-point endpoints. Keeping
  // structural zeros avoids thousands of rounded operations in derivative jets.
  if (b.lo == 0 && b.hi == 0)
    return a;
  if (a.lo == 0 && a.hi == 0)
    return b;
  return {down(a.lo + b.lo), up(a.hi + b.hi)};
}
inline Interval operator-(Interval a, Interval b) {
  if (b.lo == 0 && b.hi == 0)
    return a;
  if (a.lo == 0 && a.hi == 0)
    return {-b.hi, -b.lo};
  return {down(a.lo - b.hi), up(a.hi - b.lo)};
}
inline Interval operator*(Interval a, Interval b) {
  if ((a.lo == 0 && a.hi == 0) || (b.lo == 0 && b.hi == 0))
    return Interval(0);
  if (a.lo == 1 && a.hi == 1)
    return b;
  if (b.lo == 1 && b.hi == 1)
    return a;
  if (a.lo == -1 && a.hi == -1)
    return {-b.hi, -b.lo};
  if (b.lo == -1 && b.hi == -1)
    return {-a.hi, -a.lo};
  const double v[4] = {a.lo * b.lo, a.lo * b.hi, a.hi * b.lo, a.hi * b.hi};
  return {down(*std::min_element(v, v + 4)), up(*std::max_element(v, v + 4))};
}
inline Interval sin(Interval x) {
  if (x.hi - x.lo >= 2 * kinematic_detail::pi || std::max(std::abs(x.lo), std::abs(x.hi)) > 1e12)
    return {-1, 1};
  double l = std::min(std::sin(x.lo), std::sin(x.hi)), h = std::max(std::sin(x.lo), std::sin(x.hi));
  const double pi = kinematic_detail::pi;
  const auto begin = std::ceil((x.lo - pi / 2) / pi), end = std::floor((x.hi - pi / 2) / pi);
  for (double k = begin; k <= end; ++k) {
    const double v = std::fmod(std::abs(k), 2) < .5 ? 1 : -1;
    l = std::min(l, v);
    h = std::max(h, v);
  }
  const double pad = 32 * std::numeric_limits<double>::epsilon();
  return {std::max(-1., down(l - pad)), std::min(1., up(h + pad))};
}
inline Interval cos(Interval x) {
  return sin(x + Interval(kinematic_detail::pi / 2));
}
inline Interval operator/(Interval a, Interval b) {
  if (b.lo <= 0 && b.hi >= 0)
    throw std::domain_error("Interval denominator crosses zero");
  return a * Interval(down(1 / b.hi), up(1 / b.lo));
}
inline Interval sqrt(Interval x) {
  if (x.lo <= 0)
    throw std::domain_error("Unresolved positive square root");
  return {down(std::sqrt(x.lo)), up(std::sqrt(x.hi))};
}
using Jet = differential_math::Jet<Interval>;
inline Jet divide(const Jet &a, const Jet &x) {
  Jet y;
  y.d[0] = Interval(1) / x.d[0];
  const auto s = y.d[0];
  y.d[1] = Interval(0) - x.d[1] * s * s;
  y.d[2] = Interval(2) * x.d[1] * x.d[1] * s * s * s - x.d[2] * s * s;
  y.d[3] = Interval(0) - Interval(6) * x.d[1] * x.d[1] * x.d[1] * s * s * s * s +
           Interval(6) * x.d[1] * x.d[2] * s * s * s - x.d[3] * s * s;
  return a * y;
}
inline Jet root(const Jet &x) {
  Jet y;
  y.d[0] = sqrt(x.d[0]);
  y.d[1] = x.d[1] / (Interval(2) * y.d[0]);
  y.d[2] = (x.d[2] - Interval(2) * y.d[1] * y.d[1]) / (Interval(2) * y.d[0]);
  y.d[3] = (x.d[3] - Interval(6) * y.d[1] * y.d[2]) / (Interval(2) * y.d[0]);
  return y;
}
using JV = std::array<Jet, 3>;
inline JV plus(JV a, const JV &b) {
  for (int i = 0; i < 3; ++i)
    a[i] = a[i] + b[i];
  return a;
}
inline JV minus(JV a, const JV &b) {
  for (int i = 0; i < 3; ++i)
    a[i] = a[i] - b[i];
  return a;
}
inline JV times(JV a, const Jet &s) {
  for (auto &x : a)
    x = x * s;
  return a;
}
inline Jet dot(const JV &a, const JV &b) {
  Jet r;
  for (int i = 0; i < 3; ++i)
    r = r + a[i] * b[i];
  return r;
}
inline JV cross(const JV &a, const JV &b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

struct Result {
  std::array<Jet, 3> position;
  std::array<std::array<Interval, 3>, 3> angular;
};
inline std::vector<Jet> polynomial_jets(const std::vector<polynomial_detail::Control> &controls,
                                        long double span, double lo = 0, double hi = 1) {
  std::vector<Jet> q(controls.size());
  for (std::size_t i = 0; i < q.size(); ++i) {
    auto b = controls[i];
    for (int order = 0; order < 4; ++order) {
      const auto r = polynomial_detail::range(b, 5 - order, lo, hi);
      q[i].d[order] = Interval(down(double(r.first)), up(double(r.second)));
      if (order < 3)
        b = polynomial_detail::derivative(b, 5 - order, span);
    }
  }
  return q;
}
namespace detail {
// A constant frame has zero derivatives and every rigid transform has last row
// [0, 0, 0, 1]. Evaluate only the nonzero terms rather than a dense 4x4 jet
// product. All remaining scalar operations still use outward-rounded intervals.
inline differential_math::Matrix<Jet> right_pose(const differential_math::Matrix<Jet> &a, const Pose &pose) {
  const auto b = differential_math::pose<double>(pose);
  auto r = differential_math::identity<Jet>();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 4; ++j)
      for (int order = 0; order < 4; ++order) {
        auto value = j == 3 ? a[i][3].d[order] : Interval(0);
        for (int k = 0; k < 3; ++k)
          value = value + a[i][k].d[order] * Interval(b[k][j]);
        r[i][j].d[order] = value;
      }
  return r;
}
inline differential_math::Matrix<Jet> right_joint(const differential_math::Matrix<Jet> &a,
                                                  const SerialJoint &joint, const Jet &q) {
  auto r = a;
  if (joint.type == SerialJointType::Prismatic) {
    for (int i = 0; i < 3; ++i)
      for (int k = 0; k < 3; ++k)
        r[i][3] = r[i][3] + a[i][k] * (Jet(joint.axis[k]) * q);
  } else {
    const auto b = differential_math::rotation(joint.axis, q);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) {
        r[i][j] = Jet{};
        for (int k = 0; k < 3; ++k)
          r[i][j] = r[i][j] + a[i][k] * b[k][j];
      }
  }
  return r;
}
} // namespace detail
inline Result evaluate(const IKinematics &model, const std::vector<Jet> &q) {
  using namespace differential_math;
  if (q.size() != model.dof())
    throw std::domain_error("Enclosure dimension mismatch");
  Result r;
  if (dynamic_cast<const CartesianGantry *>(&model)) {
    for (int i = 0; i < 3; ++i)
      r.position[i] = q[i];
    return r;
  }
  if (const auto *scara = dynamic_cast<const ScaraKinematics *>(&model)) {
    const auto g = scara->geometry();
    r.position[0] = Jet(g.first_link_m) * cosine(q[0]) + Jet(g.second_link_m) * cosine(q[0] + q[1]);
    r.position[1] = Jet(g.first_link_m) * sine(q[0]) + Jet(g.second_link_m) * sine(q[0] + q[1]);
    r.position[2] = q[2];
    for (int o = 0; o < 3; ++o)
      r.angular[o][2] = q[0].d[o + 1] + q[1].d[o + 1] + q[3].d[o + 1];
    return r;
  }
  if (const auto *delta = dynamic_cast<const DeltaKinematics *>(&model)) {
    const auto g = delta->geometry();
    std::array<JV, 3> c;
    for (int i = 0; i < 3; ++i) {
      const double angle = 2 * kinematic_detail::pi * i / 3;
      auto radial = Jet(g.base_radius_m - g.tool_radius_m) + Jet(g.upper_arm_m) * cosine(q[i]);
      c[i] = {Jet(std::cos(angle)) * radial, Jet(std::sin(angle)) * radial,
              Jet(0) - Jet(g.upper_arm_m) * sine(q[i])};
    }
    const auto ab = minus(c[1], c[0]), ac = minus(c[2], c[0]);
    const auto distance = root(dot(ab, ab));
    const auto ex = times(ab, divide(Jet(1), distance));
    const auto i = dot(ex, ac);
    const auto perpendicular = minus(ac, times(ex, i));
    const auto j = root(dot(perpendicular, perpendicular));
    const auto ey = times(perpendicular, divide(Jet(1), j)), ez = cross(ex, ey);
    const auto x = Jet(.5) * distance, y = divide(i * i + j * j - Jet(2) * i * x, Jet(2) * j);
    const auto height = root(Jet(g.lower_arm_m * g.lower_arm_m) - x * x - y * y);
    if (ez[2].d[0].lo <= 0 && ez[2].d[0].hi >= 0)
      throw std::domain_error("Delta assembly sign unresolved");
    const auto middle = plus(c[0], plus(times(ex, x), times(ey, y)));
    r.position = ez[2].d[0].lo > 0 ? minus(middle, times(ez, height)) : plus(middle, times(ez, height));
    return r;
  }
#ifdef MOTION_WITH_SERIAL
  const auto *serial = dynamic_cast<const SerialChainKinematics *>(&model);
  if (!serial)
    throw std::domain_error("No restored enclosure provider for this model");
  auto t = pose<Jet>(serial->base_pose());
  std::size_t k = 0;
  for (const auto &j : serial->joints()) {
    t = detail::right_pose(t, j.origin);
    if (j.type == SerialJointType::Fixed)
      continue;
    t = detail::right_joint(t, j, q[k]);
    ++k;
  }
  t = detail::right_pose(t, serial->tool_offset());
  for (int i = 0; i < 3; ++i)
    r.position[i] = t[i][3];
  r.angular = angular(t);
  return r;
#else
  throw std::domain_error("Serial enclosure support disabled in this build");
#endif
}
} // namespace motion::enclosure
