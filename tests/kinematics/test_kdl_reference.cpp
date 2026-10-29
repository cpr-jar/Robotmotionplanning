#include "motion/io/robot_model_import.hpp"
#include <cassert>
#include <chainfksolverpos_recursive.hpp>
#include <chainjnttojacsolver.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <tinyxml2.h>
using namespace motion;
namespace {
KDL::Frame pose(const Pose &p) {
  return {KDL::Rotation::Quaternion(p.orientation[1], p.orientation[2], p.orientation[3], p.orientation[0]),
          KDL::Vector(p.position[0], p.position[1], p.position[2])};
}
KDL::Vector vector(const char *text, KDL::Vector fallback = {}) {
  if (!text)
    return fallback;
  std::istringstream in(text);
  double x, y, z;
  assert(bool(in >> x >> y >> z));
  return {x, y, z};
}
KDL::Chain reference(const std::string &xml, const SerialRobotProfile &profile) {
  tinyxml2::XMLDocument document;
  assert(document.Parse(xml.c_str()) == 0);
  std::map<std::string, const tinyxml2::XMLElement *> incoming;
  for (auto *j = document.RootElement()->FirstChildElement("joint"); j; j = j->NextSiblingElement("joint"))
    incoming[j->FirstChildElement("child")->Attribute("link")] = j;
  std::vector<const tinyxml2::XMLElement *> path;
  auto tip = profile.tip_link;
  while (tip != profile.base_link) {
    auto *j = incoming.at(tip);
    path.push_back(j);
    tip = j->FirstChildElement("parent")->Attribute("link");
  }
  KDL::Chain chain;
  chain.addSegment(KDL::Segment("world-base", KDL::Joint(KDL::Joint::Fixed), pose(profile.base_pose)));
  for (auto it = path.rbegin(); it != path.rend(); ++it) {
    const auto *j = *it;
    KDL::Frame origin;
    auto *o = j->FirstChildElement("origin");
    if (o) {
      auto xyz = vector(o->Attribute("xyz")), rpy = vector(o->Attribute("rpy"));
      origin = KDL::Frame(KDL::Rotation::RPY(rpy[0], rpy[1], rpy[2]), xyz);
    }
    const std::string type = j->Attribute("type");
    KDL::Joint joint(KDL::Joint::Fixed);
    if (type != "fixed") {
      auto *a = j->FirstChildElement("axis");
      auto axis = vector(a ? a->Attribute("xyz") : nullptr, {1, 0, 0});
      axis.Normalize();
      joint = KDL::Joint(j->Attribute("name"), origin.p, origin.M * axis,
                         type == "prismatic" ? KDL::Joint::TransAxis : KDL::Joint::RotAxis);
    }
    chain.addSegment(KDL::Segment(j->Attribute("name"), joint, origin));
  }
  chain.addSegment(KDL::Segment("tcp", KDL::Joint(KDL::Joint::Fixed), pose(profile.tool_offset)));
  return chain;
}
std::size_t cases = 0;
double max_pose = 0, max_jacobian = 0;
void accumulate_error(double actual, double expected, double &maximum) {
  assert(std::isfinite(actual) && std::isfinite(expected));
  maximum = std::max(maximum, std::abs(actual - expected));
}
void compare(const ImportedSerialRobot &robot, KDL::Chain &chain, const Vector &q) {
  KDL::JntArray joints(q.size());
  for (std::size_t i = 0; i < q.size(); ++i)
    joints(i) = q[i];
  KDL::Frame fk;
  KDL::ChainFkSolverPos_recursive solver(chain);
  assert(solver.JntToCart(joints, fk) == 0);
  KDL::Jacobian jac(q.size());
  KDL::ChainJntToJacSolver differential(chain);
  assert(differential.JntToJac(joints, jac) == 0);
  const auto actual = robot.kinematics->fk(q);
  const auto matrix = pose(actual);
  const auto J = robot.kinematics->jacobian(q);
  for (int r = 0; r < 3; ++r) {
    accumulate_error(matrix.p[r], fk.p[r], max_pose);
    for (int c = 0; c < 3; ++c)
      accumulate_error(matrix.M(r, c), fk.M(r, c), max_pose);
  }
  for (int r = 0; r < 6; ++r)
    for (std::size_t c = 0; c < q.size(); ++c)
      accumulate_error(J[r][c], jac(r, c), max_jacobian);
  assert(max_pose < 2e-11 && max_jacobian < 2e-11);
  ++cases;
}
std::string read(const std::string &path) {
  std::ifstream file(path);
  assert(file);
  return {std::istreambuf_iterator<char>(file), {}};
}
} // namespace
int main() {
  std::mt19937 random(0x4BDF01);
  std::uniform_real_distribution<double> u(0, 1), s(-1, 1);
  std::cout << "KDL reference seed=0x4BDF01\n";
  for (const auto *name : {"ur5e", "abb_irb120", "fanuc_lrmate200id"}) {
    auto directory = std::string(MOTION_SOURCE_DIR) + "/models/" + name;
    auto xml = read(directory + "/robot.urdf");
    auto profile = parse_serial_profile_yaml(read(directory + "/profile.yaml"));
    auto robot = import_serial_urdf(xml, profile);
    auto kdl = reference(xml, profile);
    for (int trial = 0; trial < 300; ++trial) {
      Vector q(6);
      for (int i = 0; i < 6; ++i)
        q[i] = robot.joint_limits.min_position[i] +
               u(random) * (robot.joint_limits.max_position[i] - robot.joint_limits.min_position[i]);
      compare(robot, kdl, q);
    }
    compare(robot, kdl, robot.joint_limits.min_position);
    compare(robot, kdl, robot.joint_limits.max_position);
  }
  for (int fixture = 0; fixture < 24; ++fixture) {
    SerialRobotProfile profile;
    profile.robot_name = "mixed";
    profile.base_link = "base";
    profile.tip_link = "l8";
    profile.base_pose =
        tracking_detail::rpy({s(random), s(random), s(random)}, {s(random), s(random), s(random)});
    profile.tool_offset = tracking_detail::rpy({.1 * s(random), .1 * s(random), .1 * s(random)},
                                               {s(random), s(random), s(random)});
    profile.tcp_limits = {1, 2, 10, 1, 2, 10};
    std::ostringstream xml;
    xml << std::setprecision(17) << "<robot name='mixed'><link name='base'/>";
    for (int i = 0; i < 9; ++i)
      xml << "<link name='l" << i << "'/>";
    std::size_t moving = 0;
    for (int i = 0; i < 9; ++i) {
      const bool fixed = i % 3 == 0;
      const std::string name = "j" + std::to_string(i);
      xml << "<joint name='" << name << "' type='" << (fixed ? "fixed" : (i % 2 ? "prismatic" : "revolute"))
          << "'><parent link='" << (i == 0 ? "base" : "l" + std::to_string(i - 1)) << "'/><child link='l" << i
          << "'/><origin xyz='" << s(random) << ' ' << s(random) << ' ' << s(random) << "' rpy='" << s(random)
          << ' ' << s(random) << ' ' << s(random) << "'/>";
      if (!fixed) {
        xml << "<axis xyz='" << s(random) << ' ' << s(random) << ' ' << s(random)
            << "'/><limit lower='-1' upper='1' velocity='1'/>";
        profile.joints[name] = {2, 10, {}, {}, {}};
        ++moving;
      }
      xml << "</joint>";
    }
    xml << "</robot>";
    auto robot = import_serial_urdf(xml.str(), profile);
    auto kdl = reference(xml.str(), profile);
    for (int trial = 0; trial < 40; ++trial) {
      Vector q(moving);
      for (auto &v : q)
        v = s(random);
      compare(robot, kdl, q);
    }
  }
  std::cout << "Independent Orocos KDL FK/J cases=" << cases
            << ", maximum pose coefficient error=" << max_pose << ", maximum Jacobian error=" << max_jacobian
            << "\n";
}
