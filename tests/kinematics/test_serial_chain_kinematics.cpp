#include "motion/io/robot_model_import.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
using namespace motion;
namespace {
void near(double a, double b, double tolerance = 1e-7) {
  if (std::abs(a - b) > tolerance) {
    std::cerr << a << " != " << b << '\n';
    std::abort();
  }
}
template <class F> void reject(F f) {
  bool threw = false;
  try {
    f();
  } catch (const std::exception &) {
    threw = true;
  }
  assert(threw);
}
void inspect(const char *name) {
  const auto directory = std::string(MOTION_SOURCE_DIR) + "/models/" + name;
  auto robot = load_serial_robot_files(directory + "/robot.urdf", directory + "/profile.yaml");
  assert(robot.kinematics->dof() == 6);
  std::ifstream file(directory + "/reference_poses.csv");
  std::string line;
  std::getline(file, line);
  while (std::getline(file, line)) {
    std::istringstream in(line);
    std::string item;
    std::getline(in, item, ',');
    Vector q(6);
    for (auto &x : q) {
      std::getline(in, item, ',');
      x = std::stod(item);
    }
    double expected[4][4];
    for (auto &row : expected)
      for (auto &x : row) {
        std::getline(in, item, ',');
        x = std::stod(item);
      }
    const auto pose = robot.kinematics->fk(q);
    for (int i = 0; i < 3; ++i)
      near(pose.position[i], expected[i][3], 1e-12);
    for (int j = 0; j < 3; ++j) {
      kinematic_detail::V3 unit{};
      unit[j] = 1;
      const auto v = tracking_detail::rotate(pose.orientation, unit);
      for (int i = 0; i < 3; ++i)
        near(v[i], expected[i][j], 1e-12);
    }
  }
  for (int trial = 0; trial < 15; ++trial) {
    Vector q{.2 + trial * .002, -.6, .7, .3, .8, -.2}, v{.07, -.04, .03, .02, -.02, .05},
        a{.02, -.01, .03, .01, -.02, .01}, j{.01, .03, -.02, .02, .01, -.01};
    const auto derivatives = robot.kinematics->differentiate(q, v, a, j);
    const auto J = robot.kinematics->jacobian(q);
    for (int r = 0; r < 6; ++r) {
      double value = 0;
      for (int c = 0; c < 6; ++c)
        value += J[r][c] * v[c];
      near(value, derivatives.velocity.data[r], 1e-12);
    }
    const double h = 1e-4;
    Vector qp = q, qm = q, vp = v, vm = v, ap = a, am = a;
    for (int i = 0; i < 6; ++i) {
      qp[i] += v[i] * h + a[i] * h * h / 2 + j[i] * h * h * h / 6;
      qm[i] -= v[i] * h - a[i] * h * h / 2 + j[i] * h * h * h / 6;
      vp[i] += a[i] * h + j[i] * h * h / 2;
      vm[i] -= a[i] * h - j[i] * h * h / 2;
      ap[i] += j[i] * h;
      am[i] -= j[i] * h;
    }
    const auto plus = robot.kinematics->differentiate(qp, vp, ap, j),
               minus = robot.kinematics->differentiate(qm, vm, am, j);
    const auto pp = robot.kinematics->fk(qp), pm = robot.kinematics->fk(qm);
    for (int r = 0; r < 6; ++r) {
      near((plus.velocity.data[r] - minus.velocity.data[r]) / (2 * h), derivatives.acceleration.data[r],
           1e-8);
      near((plus.acceleration.data[r] - minus.acceleration.data[r]) / (2 * h), derivatives.jerk.data[r],
           1e-8);
      if (r < 3)
        near((pp.position[r] - pm.position[r]) / (2 * h), derivatives.velocity.data[r], 1e-8);
    }
    auto seed = q;
    for (auto &x : seed)
      x += .01;
    const auto inverse = robot.kinematics->ik_detailed(robot.kinematics->fk(q), seed);
    if (!inverse) {
      std::cerr << name << ' ' << inverse.diagnostic << '\n';
      std::abort();
    }
    const auto recovered = robot.kinematics->fk(inverse.solution), original = robot.kinematics->fk(q);
    for (int i = 0; i < 3; ++i)
      near(recovered.position[i], original.position[i], 1e-7);
  }
  auto bad = robot.kinematics->fk({.2, -.6, .7, .3, .8, -.2});
  bad.position = {100, 0, 0};
  assert(!robot.kinematics->ik(bad, {.2, -.6, .7, .3, .8, -.2}));
  reject([&] { robot.kinematics->fk({0}); });
  std::cout << name << ": independent FK fixtures, analytic derivatives and local IK passed\n";
}
} // namespace
int main() {
  for (const auto *name : {"ur5e", "abb_irb120", "fanuc_lrmate200id"})
    inspect(name);

  const std::string profile =
      "schema_version: 1\nunits: SI\nrobot_name: fixture\nbase_link: base\ntip_link: tool\njoints: {j: "
      "{max_acceleration: 1, max_jerk: 5}}\ntcp_limits: {max_linear_velocity: 1, max_linear_acceleration: 1, "
      "max_linear_jerk: 1, max_angular_velocity: 1, max_angular_acceleration: 1, max_angular_jerk: 1}\n";
  assert(parse_serial_profile_yaml(profile).joints.size() == 1);
  reject([&] { parse_serial_profile_yaml(profile + "units: SI\n"); });
  reject([&] { parse_serial_profile_yaml(profile + "unknown: 1\n"); });
  auto tagged = profile;
  tagged.replace(tagged.find("max_acceleration: 1"), 19, "max_acceleration: !degrees 90");
  reject([&] { parse_serial_profile_yaml(tagged); });
  const std::string xml = "<robot name='fixture'><link name='base'/><link name='tool'/><joint name='j' "
                          "type='revolute'><parent link='base'/><child link='tool'/><axis xyz='0 0 "
                          "1'/><limit lower='-2' upper='2' velocity='1'/></joint></robot>";
  assert(import_serial_urdf(xml, parse_serial_profile_yaml(profile)).kinematics->dof() == 1);
  auto mimic = xml;
  mimic.insert(mimic.find("<limit"), "<mimic joint='j'/>");
  reject([&] { import_serial_urdf(mimic, parse_serial_profile_yaml(profile)); });
  const auto type = SerialJointType::Revolute;
  SerialChainKinematics pendulum({{"j", "base", "tool", type, {}, {0, 0, 1}, -3, 3, 1}}, {},
                                 {{1, 0, 0}, {1, 0, 0, 0}});
  const double q = .4, v = .2, a = .1, j = .03;
  const auto d = pendulum.differentiate({q}, {v}, {a}, {j});
  near(d.velocity.data[0], -std::sin(q) * v, 1e-12);
  near(d.acceleration.data[0], -std::cos(q) * v * v - std::sin(q) * a, 1e-12);
  near(d.jerk.data[0], std::sin(q) * v * v * v - 3 * std::cos(q) * v * a - std::sin(q) * j, 1e-12);
  near(d.jerk.data[5], j, 1e-12);
  std::cout << "Independent one-axis derivative oracle passed\n";
}
