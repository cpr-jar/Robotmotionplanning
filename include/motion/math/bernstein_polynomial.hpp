#pragma once
#include <algorithm>
#include <array>
#include <utility>
namespace motion {
namespace polynomial_detail {
using Control = std::array<long double, 6>;
inline long double value(Control b, std::size_t degree, long double u) {
  for (std::size_t n = degree; n > 0; --n)
    for (std::size_t i = 0; i < n; ++i)
      b[i] = (1 - u) * b[i] + u * b[i + 1];
  return b[0];
}
inline Control derivative(Control b, std::size_t degree, long double duration) {
  for (std::size_t i = 0; i < degree; ++i)
    b[i] = degree * (b[i + 1] - b[i]) / duration;
  return b;
}
inline std::pair<Control, Control> split_at(Control b, std::size_t degree, long double u) {
  Control l{}, r{};
  l[0] = b[0];
  r[degree] = b[degree];
  for (auto n = degree; n > 0; --n) {
    for (std::size_t i = 0; i < n; ++i)
      b[i] = (1 - u) * b[i] + u * b[i + 1];
    l[degree + 1 - n] = b[0];
    r[n - 1] = b[n - 1];
  }
  return {l, r};
}
inline std::pair<Control, Control> split(Control b) {
  return split_at(b, 5, .5L);
}
inline std::pair<long double, long double> range(Control b, std::size_t degree, long double lo,
                                                 long double hi) {
  b = split_at(b, degree, hi).first;
  b = split_at(b, degree, hi > 0 ? lo / hi : 0).second;
  const auto r = std::minmax_element(b.begin(), b.begin() + degree + 1);
  return {*r.first, *r.second};
}
inline Control hermite(long double q0, long double v0, long double a0, long double q1, long double v1,
                       long double a1, long double T) {
  return {q0,
          q0 + v0 * T / 5,
          q0 + 2 * v0 * T / 5 + a0 * T * T / 20,
          q1 - 2 * v1 * T / 5 + a1 * T * T / 20,
          q1 - v1 * T / 5,
          q1};
}
} // namespace polynomial_detail
} // namespace motion
