#pragma once
#include "kinematics.hpp"
#include <memory>
namespace motion {
using PathVector3 = std::array<double, 3>;
struct CartesianPathPoint {
  Pose pose;
  std::array<PathVector3, 3> derivatives{}, angular_derivatives{};
};
struct CartesianPathBounds {
  PathVector3 lower{}, upper{};
  std::array<PathVector3, 3> component_abs{};
  std::array<double, 3> linear_abs{};
};
class IFixedOrientationPath {
public:
  virtual ~IFixedOrientationPath() = default;
  virtual CartesianPathPoint evaluate(double) const = 0;
  virtual CartesianPathBounds bounds() const = 0;
  virtual std::array<double, 4> orientation() const = 0;
};
namespace path_detail {
inline void parameter(double s) {
  if (!std::isfinite(s) || s < 0 || s > 1)
    throw std::invalid_argument("Path parameter outside interval");
}
inline void finite3(PathVector3 p) {
  for (double x : p)
    if (!std::isfinite(x))
      throw std::invalid_argument("Nonfinite path");
}
inline void orientation(std::array<double, 4> q) {
  if (!kinematic_detail::finite_pose({{}, q}))
    throw std::invalid_argument("Invalid orientation");
}
} // namespace path_detail
class CartesianLinePath final : public IFixedOrientationPath {
public:
  CartesianLinePath(Pose first, Pose last) : first_(first), last_(last) {
    if (!kinematic_detail::finite_pose(first) || !kinematic_detail::finite_pose(last))
      throw std::invalid_argument("Invalid line pose");
    double dot = 0;
    for (int i = 0; i < 4; ++i)
      dot += first.orientation[i] * last.orientation[i];
    if (std::abs(dot) < 1 - 1e-12)
      throw std::invalid_argument("Fixed-orientation geometry requires constant orientation");
    delta_ = kinematic_detail::sub(last.position, first.position);
  }
  CartesianPathPoint evaluate(double s) const override {
    path_detail::parameter(s);
    CartesianPathPoint r;
    r.pose = {kinematic_detail::add(first_.position, kinematic_detail::scale(delta_, s)), first_.orientation};
    if (s == 1)
      r.pose.position = last_.position;
    r.derivatives[0] = delta_;
    return r;
  }
  CartesianPathBounds bounds() const override {
    CartesianPathBounds b;
    for (int i = 0; i < 3; ++i) {
      b.lower[i] = std::min(first_.position[i], last_.position[i]);
      b.upper[i] = std::max(first_.position[i], last_.position[i]);
      b.component_abs[0][i] = std::abs(delta_[i]);
    }
    b.linear_abs[0] = kinematic_detail::norm(delta_);
    return b;
  }
  std::array<double, 4> orientation() const override { return first_.orientation; }

private:
  Pose first_, last_;
  PathVector3 delta_{};
};
class CartesianArcPath final : public IFixedOrientationPath {
public:
  CartesianArcPath(PathVector3 center, Pose first, PathVector3 normal, double sweep)
      : center_(center), orientation_(first.orientation), sweep_(sweep) {
    using namespace kinematic_detail;
    path_detail::finite3(center);
    path_detail::finite3(normal);
    if (!finite_pose(first) || !std::isfinite(sweep) || sweep == 0 || std::abs(sweep) > 2 * pi)
      throw std::invalid_argument("Invalid CIRC sweep/pose");
    a_ = sub(first.position, center);
    radius_ = norm(a_);
    const double n = norm(normal);
    if (radius_ <= 0 || n <= 0 || !std::isfinite(radius_))
      throw std::invalid_argument("Degenerate arc");
    normal = scale(normal, 1 / n);
    if (std::abs(dot(scale(a_, 1 / radius_), normal)) > 1e-12)
      throw std::invalid_argument("Arc normal must be perpendicular to start radius");
    b_ = cross(normal, a_);
  }
  CartesianPathPoint evaluate(double s) const override {
    using namespace kinematic_detail;
    path_detail::parameter(s);
    const double x = s * sweep_;
    auto radial = add(scale(a_, std::cos(x)), scale(b_, std::sin(x))),
         tangent = add(scale(a_, -std::sin(x)), scale(b_, std::cos(x)));
    CartesianPathPoint r;
    r.pose = {add(center_, radial), orientation_};
    r.derivatives = {scale(tangent, sweep_), scale(radial, -sweep_ * sweep_),
                     scale(tangent, -sweep_ * sweep_ * sweep_)};
    return r;
  }
  CartesianPathBounds bounds() const override {
    CartesianPathBounds b;
    for (int i = 0; i < 3; ++i) {
      b.lower[i] = center_[i] - radius_;
      b.upper[i] = center_[i] + radius_;
      for (int o = 0; o < 3; ++o)
        b.component_abs[o][i] = radius_ * std::pow(std::abs(sweep_), o + 1) * (1 + 1e-12);
    }
    for (int o = 0; o < 3; ++o)
      b.linear_abs[o] = radius_ * std::pow(std::abs(sweep_), o + 1) * (1 + 1e-12);
    return b;
  }
  std::array<double, 4> orientation() const override { return orientation_; }

private:
  PathVector3 center_, a_, b_;
  std::array<double, 4> orientation_;
  double sweep_, radius_;
};
struct JointPathPoint {
  Vector q, q_s, q_ss, q_sss;
  CartesianPathPoint tcp;
};
struct JointPathBounds {
  Vector lower, upper;
  std::array<Vector, 3> joint_abs;
  std::array<double, 3> linear_abs{}, angular_abs{};
};
class IBoundedJointPath {
public:
  virtual ~IBoundedJointPath() = default;
  virtual JointPathPoint evaluate(double) const = 0;
  virtual JointPathBounds bounds() const = 0;
};
} // namespace motion
