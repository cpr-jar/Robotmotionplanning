#pragma once
#include "motion/core/motion_types.hpp"
#include <numbers>
namespace motion {
namespace kinematic_detail {
using V3 = std::array<double, 3>;
constexpr double pi = std::numbers::pi;
inline V3 add(V3 a, V3 b) {
  for (int i = 0; i < 3; ++i)
    a[i] += b[i];
  return a;
}
inline V3 sub(V3 a, V3 b) {
  for (int i = 0; i < 3; ++i)
    a[i] -= b[i];
  return a;
}
inline V3 scale(V3 a, double s) {
  for (auto &x : a)
    x *= s;
  return a;
}
inline double dot(V3 a, V3 b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline double norm(V3 a) {
  return std::hypot(a[0], a[1], a[2]);
}
inline V3 cross(V3 a, V3 b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
inline bool finite(const Vector &q, std::size_t n) {
  return q.size() == n && std::all_of(q.begin(), q.end(), [](double x) { return std::isfinite(x); });
}
inline bool finite_pose(const Pose &p) {
  for (double x : p.position)
    if (!std::isfinite(x))
      return false;
  double s = 0;
  for (double x : p.orientation) {
    if (!std::isfinite(x))
      return false;
    s += x * x;
  }
  return std::abs(s - 1) < 1e-9;
}
inline bool fixed_orientation(const Pose &p) {
  return finite_pose(p) && std::abs(std::abs(p.orientation[0]) - 1) < 1e-9 &&
         std::hypot(p.orientation[1], p.orientation[2], p.orientation[3]) < 1e-9;
}
inline void require_finite(const KinematicDerivatives &d) {
  for (auto t : {d.velocity, d.acceleration, d.jerk})
    for (double x : t.data)
      if (!std::isfinite(x))
        throw std::domain_error("Derivative overflow");
}
} // namespace kinematic_detail
namespace tracking_detail {
inline std::array<double, 4> normalize(std::array<double, 4> q) {
  double n = 0;
  for (double x : q) {
    if (!std::isfinite(x))
      throw std::invalid_argument("Invalid quaternion");
    n += x * x;
  }
  n = std::sqrt(n);
  if (n < 1e-15)
    throw std::invalid_argument("Zero quaternion");
  for (auto &x : q)
    x /= n;
  return q;
}
inline std::array<double, 4> multiply(std::array<double, 4> a, std::array<double, 4> b) {
  return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
          a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
          a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
          a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}
inline kinematic_detail::V3 rotate(std::array<double, 4> q, kinematic_detail::V3 p) {
  q = normalize(q);
  auto r = multiply(multiply(q, {0, p[0], p[1], p[2]}), {q[0], -q[1], -q[2], -q[3]});
  return {r[1], r[2], r[3]};
}
inline Pose compose(const Pose &a, const Pose &b) {
  return {kinematic_detail::add(a.position, rotate(a.orientation, b.position)),
          normalize(multiply(a.orientation, b.orientation))};
}
inline Pose rpy(kinematic_detail::V3 xyz, kinematic_detail::V3 r) {
  auto qx = std::array<double, 4>{std::cos(r[0] / 2), std::sin(r[0] / 2), 0, 0};
  auto qy = std::array<double, 4>{std::cos(r[1] / 2), 0, std::sin(r[1] / 2), 0};
  auto qz = std::array<double, 4>{std::cos(r[2] / 2), 0, 0, std::sin(r[2] / 2)};
  return {xyz, normalize(multiply(multiply(qz, qy), qx))};
}
} // namespace tracking_detail
} // namespace motion
