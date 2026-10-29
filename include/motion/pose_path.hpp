#pragma once
#include "boundary_polynomial.hpp"
#include "path.hpp"
namespace motion {
inline double rotation_distance(std::array<double, 4> a, std::array<double, 4> b) {
  a = tracking_detail::normalize(a);
  b = tracking_detail::normalize(b);
  a[1] = -a[1];
  a[2] = -a[2];
  a[3] = -a[3];
  const auto r = tracking_detail::multiply(a, b);
  return 2 * std::atan2(std::hypot(r[1], r[2], r[3]), std::abs(r[0]));
}
inline std::array<double, 4> smooth_rotation(std::array<double, 4> a, std::array<double, 4> b, double u) {
  a = tracking_detail::normalize(a);
  b = tracking_detail::normalize(b);
  double dot = 0;
  for (int i = 0; i < 4; ++i)
    dot += a[i] * b[i];
  if (dot < 0) {
    for (auto &x : b)
      x = -x;
    dot = -dot;
  }
  const double s = u * u * u * (10 + u * (-15 + 6 * u)), theta = std::acos(std::clamp(dot, 0., 1.));
  std::array<double, 4> r;
  for (int i = 0; i < 4; ++i)
    r[i] = theta < 1e-8 ? (1 - s) * a[i] + s * b[i]
                        : (std::sin((1 - s) * theta) * a[i] + std::sin(s * theta) * b[i]) / std::sin(theta);
  return tracking_detail::normalize(r);
}
class IPosePath {
public:
  virtual ~IPosePath() = default;
  virtual Pose evaluate(double) const = 0;
  virtual double linear_speed_bound() const noexcept = 0;
  virtual double angular_speed_bound() const noexcept = 0;
  virtual std::size_t pieces() const noexcept { return 1; }
};
class PoseSweepPath final : public IPosePath {
public:
  PoseSweepPath(std::shared_ptr<const IFixedOrientationPath> geometry, std::array<double, 4> goal)
      : geometry_(std::move(geometry)), goal_(goal) {
    if (!geometry_)
      throw std::invalid_argument("Missing pose geometry");
    path_detail::orientation(goal);
    angle_ = rotation_distance(geometry_->orientation(), goal);
    linear_ = geometry_->bounds().linear_abs[0];
  }
  Pose evaluate(double s) const override {
    auto r = geometry_->evaluate(s).pose;
    r.orientation = smooth_rotation(geometry_->orientation(), goal_, s);
    return r;
  }
  double linear_speed_bound() const noexcept override { return linear_; }
  double angular_speed_bound() const noexcept override { return angle_ * 1.875; }

private:
  std::shared_ptr<const IFixedOrientationPath> geometry_;
  std::array<double, 4> goal_;
  double angle_, linear_;
};

class BlendedPosePath final : public IPosePath {
public:
  BlendedPosePath(std::vector<Pose> points, double tolerance) : points_(std::move(points)) {
    using namespace kinematic_detail;
    if (points_.size() < 2 || points_.size() > 256 || !std::isfinite(tolerance) || tolerance <= 0)
      throw std::invalid_argument("Invalid blend points/tolerance");
    for (auto p : points_)
      if (!finite_pose(p))
        throw std::invalid_argument("Invalid blend pose");
    const auto n = points_.size() - 1;
    std::vector<PathVector3> tangents(n + 1);
    tangents[0] = scale(sub(points_[1].position, points_[0].position), n);
    tangents[n] = scale(sub(points_[n].position, points_[n - 1].position), n);
    for (std::size_t i = 1; i < n; ++i) {
      auto before = sub(points_[i].position, points_[i - 1].position),
           after = sub(points_[i + 1].position, points_[i].position);
      const double a = norm(before), b = norm(after);
      if (a == 0 || b == 0)
        continue;
      const auto sum = add(scale(before, 1 / a), scale(after, 1 / b));
      const double length = norm(sum);
      if (length < 1e-8)
        continue;
      tangents[i] = scale(sum, std::min({a, b, 2.5 * tolerance}) * n / length);
    }
    for (std::size_t k = 0; k < n; ++k) {
      std::array<polynomial_detail::Control, 3> b;
      double norm_squared = 0;
      for (int axis = 0; axis < 3; ++axis) {
        b[axis] =
            polynomial_detail::hermite(points_[k].position[axis], tangents[k][axis], 0,
                                       points_[k + 1].position[axis], tangents[k + 1][axis], 0, 1.L / n);
        const auto d = polynomial_detail::derivative(b[axis], 5, 1.L / n);
        long double peak = 0;
        for (int i = 0; i < 5; ++i)
          peak = std::max(peak, std::abs(d[i]));
        norm_squared += double(peak * peak);
      }
      linear_ = std::max(linear_, std::sqrt(norm_squared));
      angular_ = std::max(angular_,
                          rotation_distance(points_[k].orientation, points_[k + 1].orientation) * 1.875 * n);
      controls_.push_back(b);
    }
  }
  Pose evaluate(double s) const override {
    path_detail::parameter(s);
    const auto n = controls_.size(), k = std::min(n - 1, std::size_t(s * n));
    const long double u = s == 1 ? 1 : static_cast<long double>(s) * n - k;
    Pose out;
    for (int i = 0; i < 3; ++i)
      out.position[i] = double(polynomial_detail::value(controls_[k][i], 5, u));
    out.orientation = smooth_rotation(points_[k].orientation, points_[k + 1].orientation, double(u));
    return out;
  }
  double linear_speed_bound() const noexcept override { return linear_; }
  double angular_speed_bound() const noexcept override { return angular_; }
  std::size_t pieces() const noexcept override { return controls_.size(); }

private:
  std::vector<Pose> points_;
  std::vector<std::array<polynomial_detail::Control, 3>> controls_;
  double linear_{}, angular_{};
};
} // namespace motion
