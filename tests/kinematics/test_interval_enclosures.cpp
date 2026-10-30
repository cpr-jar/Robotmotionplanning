#include "motion/kinematics/interval_kinematics.hpp"
#include <cassert>
#include <iostream>
#include <random>
using namespace motion;
namespace {
void contains(enclosure::Interval interval, long double value) {
  assert(static_cast<long double>(interval.lo) <= value);
  assert(value <= static_cast<long double>(interval.hi));
}
void arithmetic() {
  using enclosure::Interval;
  const double tiny = std::numeric_limits<double>::denorm_min();
  // A long-double oracle includes underflow, signed zeros and exact identities.
  const double values[]{-1e150, -2., -1., -tiny, -0., 0., tiny, 1., 2., 1e150};
  for (double a : values)
    for (double b : values) {
      contains(Interval(a) + Interval(b), static_cast<long double>(a) + b);
      contains(Interval(a) - Interval(b), static_cast<long double>(a) - b);
      contains(Interval(a) * Interval(b), static_cast<long double>(a) * b);
    }
  std::mt19937 random(0xEAC105E);
  std::uniform_real_distribution<double> sample(-100, 100), unit(0, 1);
  for (int trial = 0; trial < 2000; ++trial) {
    const double a = sample(random), b = a + std::abs(sample(random));
    const double c = sample(random), d = c + std::abs(sample(random));
    const Interval x(a, b), y(c, d);
    for (double u : {0., 1., unit(random)})
      for (double v : {0., 1., unit(random)}) {
        // Use convex combinations of endpoints, without rounded b-a.
        const long double exact_p = (1.L - u) * a + static_cast<long double>(u) * b;
        const long double exact_q = (1.L - v) * c + static_cast<long double>(v) * d;
        contains(x + y, exact_p + exact_q);
        contains(x - y, exact_p - exact_q);
        contains(x * y, exact_p * exact_q);
      }
  }
}
void mixed_chains() {
  std::mt19937 random(0xC0FFEE);
  std::uniform_real_distribution<double> value(-.8, .8), unit(-1, 1);
  const auto frame = [&]() {
    return tracking_detail::rpy({value(random), value(random), value(random)},
                                {value(random), value(random), value(random)});
  };
  for (int trial = 0; trial < 32; ++trial) {
    std::vector<SerialJoint> joints;
    for (int k = 0; k < 8; ++k) {
      const auto type = k % 4 == 0   ? SerialJointType::Fixed
                        : k % 4 == 1 ? SerialJointType::Prismatic
                        : k % 4 == 2 ? SerialJointType::Revolute
                                     : SerialJointType::Continuous;
      joints.push_back({"j" + std::to_string(k),
                        "link" + std::to_string(k),
                        "link" + std::to_string(k + 1),
                        type,
                        frame(),
                        {value(random), value(random), value(random)},
                        -3,
                        3,
                        1});
    }
    const SerialChainKinematics model(joints, frame(), frame());
    std::vector<enclosure::Jet> box(model.dof());
    for (auto &axis : box)
      for (auto &order : axis.d) {
        const double center = value(random);
        order = enclosure::Interval(center - .01, center + .01);
      }
    const auto bounds = enclosure::evaluate(model, box);
    for (int sample = 0; sample < 25; ++sample) {
      std::array<Vector, 4> state;
      for (auto &v : state)
        v.resize(model.dof());
      for (std::size_t k = 0; k < model.dof(); ++k)
        for (int order = 0; order < 4; ++order) {
          const auto interval = box[k].d[order];
          state[order][k] = (interval.lo + interval.hi) / 2 + unit(random) * .009;
        }
      const auto pose = model.fk(state[0]);
      // The reference uses Eigen FK and the unchanged dense double-jet algebra,
      // independently of the optimized interval composition.
      const auto d = model.differentiate(state[0], state[1], state[2], state[3]);
      const Twist derivatives[]{d.velocity, d.acceleration, d.jerk};
      for (int i = 0; i < 3; ++i) {
        contains(bounds.position[i].d[0], pose.position[i]);
        for (int order = 0; order < 3; ++order) {
          contains(bounds.position[i].d[order + 1], derivatives[order].data[i]);
          contains(bounds.angular[order][i], derivatives[order].data[i + 3]);
        }
      }
    }
  }
}
} // namespace
int main() {
  arithmetic();
  mixed_chains();
  std::cout
      << "Interval identities: extended-precision oracle; 800 mixed-chain FK/derivative states enclosed\n";
}
