#include "motion/model_import.hpp"
#include <fstream>
#include <locale>
#include <set>
#include <sstream>
#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>
namespace motion {
namespace {
[[noreturn]] void fail(const std::string &s) {
  throw ModelImportError(s);
}
void yaml_tag(const YAML::Node &n) {
  const auto t = n.Tag();
  if (!t.empty() && t != "?" && t != "!")
    fail("Explicit YAML tags unsupported; convert to SI");
}
void keys(const YAML::Node &n, std::initializer_list<const char *> allowed) {
  yaml_tag(n);
  if (!n.IsMap())
    fail("Expected YAML mapping");
  std::set<std::string> seen;
  for (const auto &p : n) {
    if (!p.first.IsScalar())
      fail("Invalid YAML key");
    yaml_tag(p.first);
    const auto s = p.first.as<std::string>();
    if (!seen.insert(s).second)
      fail("Duplicate key: " + s);
    if (std::none_of(allowed.begin(), allowed.end(), [&](const char *k) { return s == k; }))
      fail("Unknown key: " + s);
  }
}
double number(const YAML::Node &n, bool positive = false) {
  yaml_tag(n);
  if (!n.IsScalar())
    fail("Expected finite YAML number");
  const double x = n.as<double>();
  if (!std::isfinite(x) || (positive && x <= 0))
    fail("Invalid numeric limit");
  return x;
}
std::string text(const YAML::Node &n) {
  yaml_tag(n);
  if (!n.IsScalar() || n.as<std::string>().empty())
    fail("Expected YAML text");
  return n.as<std::string>();
}
kinematic_detail::V3 xyz(const YAML::Node &n) {
  yaml_tag(n);
  if (!n.IsSequence() || n.size() != 3)
    fail("Expected three SI numbers");
  return {number(n[0]), number(n[1]), number(n[2])};
}
Pose pose(const YAML::Node &n) {
  if (!n)
    return {};
  keys(n, {"xyz", "rpy"});
  return tracking_detail::rpy(n["xyz"] ? xyz(n["xyz"]) : kinematic_detail::V3{},
                              n["rpy"] ? xyz(n["rpy"]) : kinematic_detail::V3{});
}
std::string read(const std::string &p, std::size_t cap) {
  std::ifstream f(p, std::ios::binary | std::ios::ate);
  if (!f)
    fail("Cannot open model: " + p);
  auto n = f.tellg();
  if (n < 0 || static_cast<unsigned long long>(n) > cap)
    fail("Model too large: " + p);
  std::string s(std::size_t(n), '\0');
  f.seekg(0);
  if (!f.read(s.data(), std::streamsize(s.size())))
    fail("Cannot read model: " + p);
  return s;
}
const char *attr(const tinyxml2::XMLElement *e, const char *name) {
  if (!e || !e->Attribute(name))
    fail(std::string("Missing URDF attribute: ") + name);
  return e->Attribute(name);
}
double numeric(const char *s) {
  if (!s)
    fail("Missing URDF number");
  std::istringstream in(s);
  in.imbue(std::locale::classic());
  double x;
  if (!(in >> x) || !std::isfinite(x))
    fail("Nonfinite URDF number");
  in >> std::ws;
  if (!in.eof())
    fail("Trailing numeric URDF content");
  return x;
}
kinematic_detail::V3 vector(const char *s, kinematic_detail::V3 fallback = {}) {
  if (!s)
    return fallback;
  std::istringstream in(s);
  in.imbue(std::locale::classic());
  kinematic_detail::V3 v;
  for (auto &x : v)
    if (!(in >> x) || !std::isfinite(x))
      fail("Invalid URDF vector");
  in >> std::ws;
  if (!in.eof())
    fail("Expected exactly 3 URDF values");
  return v;
}
const tinyxml2::XMLElement *child(const tinyxml2::XMLElement *e, const char *n) {
  const auto *p = e->FirstChildElement(n);
  if (p && p->NextSiblingElement(n))
    fail(std::string("Duplicate URDF element: ") + n);
  return p;
}
void inspect(const tinyxml2::XMLElement *e, std::size_t depth = 0) {
  if (depth > 64)
    fail("URDF nesting budget");
  if (std::string_view(e->Name()).find(':') != std::string_view::npos)
    fail("Expand Xacro offline first");
  for (auto *a = e->FirstAttribute(); a; a = a->Next()) {
    auto s = std::string_view(a->Value());
    if (s.find("${") != s.npos || s.find("$(") != s.npos)
      fail("Unexpanded URDF expression");
  }
  for (auto *c = e->FirstChildElement(); c; c = c->NextSiblingElement())
    inspect(c, depth + 1);
}
} // namespace
SerialRobotProfile parse_serial_profile_yaml(std::string_view content) {
  if (content.empty() || content.size() > 128 * 1024 || content.find('\0') != content.npos)
    fail("Invalid profile size/text");
  try {
    const auto documents = YAML::LoadAll(std::string(content));
    if (documents.size() != 1)
      fail("Expected one profile document");
    const auto n = documents[0];
    keys(n, {"schema_version", "units", "robot_name", "base_link", "tip_link", "base_pose", "tool_offset",
             "joints", "tcp_limits", "solver"});
    if (number(n["schema_version"]) != 1 || text(n["units"]) != "SI")
      fail("Expected schema 1 with units SI");
    SerialRobotProfile p;
    p.robot_name = text(n["robot_name"]);
    p.base_link = text(n["base_link"]);
    p.tip_link = text(n["tip_link"]);
    if (p.base_link == p.tip_link)
      fail("Empty selected chain");
    p.base_pose = pose(n["base_pose"]);
    p.tool_offset = pose(n["tool_offset"]);
    auto joints = n["joints"];
    if (!joints.IsMap() || joints.size() == 0 || joints.size() > 12)
      fail("Expected 1..12 joint profiles");
    for (auto field : joints) {
      const auto name = text(field.first);
      if (p.joints.count(name))
        fail("Duplicate profile joint: " + name);
      const auto j = field.second;
      keys(j, {"max_acceleration", "max_jerk", "lower", "upper", "max_velocity"});
      JointProfile v;
      v.max_acceleration = number(j["max_acceleration"], true);
      v.max_jerk = number(j["max_jerk"], true);
      if (j["lower"])
        v.lower = number(j["lower"]);
      if (j["upper"])
        v.upper = number(j["upper"]);
      if (j["max_velocity"])
        v.max_velocity = number(j["max_velocity"], true);
      p.joints.emplace(name, v);
    }
    const auto caps = n["tcp_limits"];
    keys(caps, {"max_linear_velocity", "max_linear_acceleration", "max_linear_jerk", "max_angular_velocity",
                "max_angular_acceleration", "max_angular_jerk"});
    p.tcp_limits = {
        number(caps["max_linear_velocity"], true),      number(caps["max_linear_acceleration"], true),
        number(caps["max_linear_jerk"], true),          number(caps["max_angular_velocity"], true),
        number(caps["max_angular_acceleration"], true), number(caps["max_angular_jerk"], true)};
    if (n["solver"]) {
      const auto s = n["solver"];
      keys(s, {"max_iterations", "position_tolerance_m", "orientation_tolerance_rad", "orientation_length_m",
               "prismatic_scale_m", "damping", "max_normalized_step", "singularity_threshold"});
      if (s["max_iterations"]) {
        const double v = number(s["max_iterations"], true);
        if (v != std::floor(v) || v > 10000)
          fail("Invalid solver iterations");
        p.solver.max_iterations = std::size_t(v);
      }
      for (auto pair : std::initializer_list<std::pair<const char *, double *>>{
               {"position_tolerance_m", &p.solver.position_tolerance_m},
               {"orientation_tolerance_rad", &p.solver.orientation_tolerance_rad},
               {"orientation_length_m", &p.solver.orientation_length_m},
               {"prismatic_scale_m", &p.solver.prismatic_scale_m},
               {"damping", &p.solver.damping},
               {"max_normalized_step", &p.solver.max_normalized_step},
               {"singularity_threshold", &p.solver.singularity_threshold}})
        if (s[pair.first])
          *pair.second = number(s[pair.first], true);
    }
    return p;
  } catch (const ModelImportError &) {
    throw;
  } catch (const YAML::Exception &e) {
    fail(std::string("Profile YAML: ") + e.what());
  }
}
ImportedSerialRobot import_serial_urdf(std::string_view xml, const SerialRobotProfile &p) {
  if (xml.empty() || xml.size() > 8 * 1024 * 1024 || xml.find('\0') != xml.npos ||
      xml.find("<!DOCTYPE") != xml.npos)
    fail("Invalid/unsupported URDF content");
  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml.data(), xml.size()) != tinyxml2::XML_SUCCESS)
    fail("Invalid URDF XML");
  const auto *root = doc.RootElement();
  if (!root || std::string_view(root->Name()) != "robot")
    fail("Expected URDF robot");
  inspect(root);
  std::set<std::string> links, jointnames;
  for (auto *l = root->FirstChildElement("link"); l; l = l->NextSiblingElement("link"))
    if (!links.insert(attr(l, "name")).second)
      fail("Duplicate URDF link");
  if (!links.count(p.base_link) || !links.count(p.tip_link))
    fail("Missing selected base/tip");
  std::map<std::string, SerialJoint> incoming;
  for (auto *j = root->FirstChildElement("joint"); j; j = j->NextSiblingElement("joint")) {
    SerialJoint s;
    s.name = attr(j, "name");
    if (!jointnames.insert(s.name).second)
      fail("Duplicate URDF joint");
    const std::string type = attr(j, "type");
    if (type == "fixed")
      s.type = SerialJointType::Fixed;
    else if (type == "revolute")
      s.type = SerialJointType::Revolute;
    else if (type == "continuous")
      s.type = SerialJointType::Continuous;
    else if (type == "prismatic")
      s.type = SerialJointType::Prismatic;
    else
      fail("Unsupported joint type: " + type);
    s.parent_link = attr(child(j, "parent"), "link");
    s.child_link = attr(child(j, "child"), "link");
    if (!links.count(s.parent_link) || !links.count(s.child_link) || s.parent_link == s.child_link)
      fail("Invalid joint links");
    if (auto *o = child(j, "origin"))
      s.origin = tracking_detail::rpy(vector(o->Attribute("xyz")), vector(o->Attribute("rpy")));
    if (auto *a = child(j, "axis"))
      s.axis = vector(attr(a, "xyz"));
    if (child(j, "mimic") || child(j, "calibration") || child(j, "safety_controller"))
      fail("Unsupported joint coupling/calibration/safety tag");
    if (s.type != SerialJointType::Fixed) {
      const auto *limit = child(j, "limit");
      s.max_velocity = numeric(attr(limit, "velocity"));
      if (s.type != SerialJointType::Continuous) {
        s.lower = numeric(attr(limit, "lower"));
        s.upper = numeric(attr(limit, "upper"));
      } else {
        s.lower = -INFINITY;
        s.upper = INFINITY;
      }
    }
    if (!incoming.emplace(s.child_link, s).second)
      fail("URDF child has multiple parents");
  }
  std::vector<SerialJoint> chain;
  std::string cursor = p.tip_link;
  std::set<std::string> seen;
  while (cursor != p.base_link) {
    if (!seen.insert(cursor).second || !incoming.count(cursor))
      fail("Selected chain cyclic/disconnected");
    chain.push_back(incoming.at(cursor));
    cursor = chain.back().parent_link;
  }
  std::reverse(chain.begin(), chain.end());
  JointLimits caps;
  std::set<std::string> selected;
  for (auto &j : chain)
    if (j.type != SerialJointType::Fixed) {
      if (!p.joints.count(j.name))
        fail("Missing explicit dynamics for " + j.name);
      selected.insert(j.name);
      const auto &v = p.joints.at(j.name);
      if (v.lower) {
        if (*v.lower < j.lower)
          fail("Profile expands URDF lower bound");
        j.lower = *v.lower;
      }
      if (v.upper) {
        if (*v.upper > j.upper)
          fail("Profile expands URDF upper bound");
        j.upper = *v.upper;
      }
      if (v.max_velocity) {
        if (*v.max_velocity > j.max_velocity)
          fail("Profile expands velocity limit");
        j.max_velocity = *v.max_velocity;
      }
      caps.min_position.push_back(j.lower);
      caps.max_position.push_back(j.upper);
      caps.max_velocity.push_back(j.max_velocity);
      caps.max_acceleration.push_back(v.max_acceleration);
      caps.max_jerk.push_back(v.max_jerk);
    }
  if (selected.size() != p.joints.size())
    fail("Unknown/extraneous profile joint");
  check_limits(caps, selected.size());
  return {p.robot_name, p.base_link,
          p.tip_link,   std::make_shared<SerialChainKinematics>(chain, p.base_pose, p.tool_offset, p.solver),
          caps,         p.tcp_limits};
}
ImportedSerialRobot load_serial_robot_files(const std::string &urdf, const std::string &profile) {
  return import_serial_urdf(read(urdf, 8 * 1024 * 1024),
                            parse_serial_profile_yaml(read(profile, 128 * 1024)));
}
} // namespace motion
