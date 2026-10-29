#pragma once
#include "kinematics.hpp"
namespace motion::differential_math {
// Raw derivatives, not Taylor coefficients. Scalar can also be an interval.
template <class S> struct Jet {
  std::array<S, 4> d{};
  Jet() = default;
  Jet(double x) { d[0] = S(x); }
  explicit Jet(std::array<S, 4> x) : d(x) {}
};
template <class S> Jet<S> operator+(Jet<S> a, const Jet<S> &b) {
  for (int i = 0; i < 4; ++i)
    a.d[i] = a.d[i] + b.d[i];
  return a;
}
template <class S> Jet<S> operator-(Jet<S> a, const Jet<S> &b) {
  for (int i = 0; i < 4; ++i)
    a.d[i] = a.d[i] - b.d[i];
  return a;
}
template <class S> Jet<S> operator*(const Jet<S> &a, const Jet<S> &b) {
  Jet<S> r;
  r.d[0] = a.d[0] * b.d[0];
  r.d[1] = a.d[1] * b.d[0] + a.d[0] * b.d[1];
  r.d[2] = a.d[2] * b.d[0] + S(2) * a.d[1] * b.d[1] + a.d[0] * b.d[2];
  r.d[3] = a.d[3] * b.d[0] + S(3) * a.d[2] * b.d[1] + S(3) * a.d[1] * b.d[2] + a.d[0] * b.d[3];
  return r;
}
template <class S> Jet<S> sine(const Jet<S> &x) {
  using std::cos;
  using std::sin;
  const auto s = sin(x.d[0]), c = cos(x.d[0]);
  return Jet<S>({s, c * x.d[1], c * x.d[2] - s * x.d[1] * x.d[1],
                 c * x.d[3] - S(3) * s * x.d[1] * x.d[2] - c * x.d[1] * x.d[1] * x.d[1]});
}
template <class S> Jet<S> cosine(const Jet<S> &x) {
  using std::cos;
  using std::sin;
  const auto s = sin(x.d[0]), c = cos(x.d[0]);
  return Jet<S>({c, S(0) - s * x.d[1], S(0) - s * x.d[2] - c * x.d[1] * x.d[1],
                 S(0) - s * x.d[3] - S(3) * c * x.d[1] * x.d[2] + s * x.d[1] * x.d[1] * x.d[1]});
}
template <class T> using Matrix = std::array<std::array<T, 4>, 4>;
template <class T> Matrix<T> identity() {
  Matrix<T> r{};
  for (int i = 0; i < 4; ++i)
    r[i][i] = T(1);
  return r;
}
template <class T> Matrix<T> multiply(const Matrix<T> &a, const Matrix<T> &b) {
  Matrix<T> r{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      for (int k = 0; k < 4; ++k)
        r[i][j] = r[i][j] + a[i][k] * b[k][j];
  return r;
}
template <class T> Matrix<T> pose(const Pose &p) {
  auto r = identity<T>();
  const auto q = p.orientation;
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  const double R[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
                          {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
                          {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j)
      r[i][j] = T(R[i][j]);
    r[i][3] = T(p.position[i]);
  }
  return r;
}
// Explicit skew matrix avoids axis-index ambiguity.
template <class S> Matrix<Jet<S>> rotation(std::array<double, 3> a, const Jet<S> &q) {
  const auto s = sine(q), c = cosine(q);
  auto r = identity<Jet<S>>();
  const double K[3][3] = {{0, -a[2], a[1]}, {a[2], 0, -a[0]}, {-a[1], a[0], 0}};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      r[i][j] = Jet<S>(a[i] * a[j]) * (Jet<S>(1) - c) + Jet<S>(i == j ? 1 : 0) * c + Jet<S>(K[i][j]) * s;
  return r;
}
template <class S> std::array<std::array<S, 3>, 3> angular(const Matrix<Jet<S>> &t) {
  std::array<std::array<S, 3>, 3> out{};
  for (int order = 0; order < 3; ++order) {
    std::array<std::array<S, 3>, 3> w{};
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        for (int k = 0; k < 3; ++k) {
          w[i][j] = w[i][j] + t[i][k].d[order + 1] * t[j][k].d[0];
          if (order >= 1)
            w[i][j] = w[i][j] + S(order) * t[i][k].d[order] * t[j][k].d[1];
          if (order == 2)
            w[i][j] = w[i][j] + t[i][k].d[1] * t[j][k].d[2];
        }
    out[order] = {(w[2][1] - w[1][2]) * S(.5), (w[0][2] - w[2][0]) * S(.5), (w[1][0] - w[0][1]) * S(.5)};
  }
  return out;
}
} // namespace motion::differential_math
