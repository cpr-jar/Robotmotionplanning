#include "motion/robot_program.hpp"
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <yaml-cpp/eventhandler.h>
#include <yaml-cpp/parser.h>
#include <yaml-cpp/yaml.h>
namespace motion {
namespace {
std::string describe(const ProgramLocation &l, const std::string &m) {
  return l.file + ":" + std::to_string(l.line) + ":" + std::to_string(l.column) + " [" + l.block +
         (l.step ? " step " + std::to_string(l.step) : "") + "]: " + m;
}
struct Reader {
  std::string file;
  ProgramLocation location(const YAML::Node &n, std::string block = "program", std::size_t step = 0) const {
    const auto m = n.Mark();
    return {file, std::move(block), m.is_null() ? 0 : std::size_t(m.line + 1),
            m.is_null() ? 0 : std::size_t(m.column + 1), step};
  }
  [[noreturn]] void fail(const YAML::Node &n, const std::string &m) const {
    throw RobotProgramError(location(n), m);
  }
  std::string text(const YAML::Node &n) const {
    if (!n.IsScalar())
      fail(n, "Expected text");
    const auto s = n.as<std::string>();
    if (s.empty() || s.size() > 4096)
      fail(n, "Invalid text length");
    return s;
  }
  void keys(const YAML::Node &n, std::initializer_list<const char *> allowed) const {
    if (!n.IsMap())
      fail(n, "Expected mapping");
    std::set<std::string> seen;
    for (auto f : n) {
      const auto key = text(f.first);
      if (!seen.insert(key).second)
        fail(f.first, "Duplicate key: " + key);
      if (std::none_of(allowed.begin(), allowed.end(), [&](const char *k) { return key == k; }))
        fail(f.first, "Unknown key: " + key);
    }
  }
  double number(const YAML::Node &n) const {
    if (!n.IsScalar())
      fail(n, "Expected finite number");
    double x;
    try {
      x = n.as<double>();
    } catch (const YAML::Exception &) {
      fail(n, "Expected finite number");
    }
    if (!std::isfinite(x))
      fail(n, "Expected finite number");
    return x;
  }
  double positive(const YAML::Node &n) const {
    const auto x = number(n);
    if (x <= 0)
      fail(n, "Expected positive number");
    return x;
  }
  double speed(const YAML::Node &n) const {
    const auto x = positive(n);
    if (x > 1)
      fail(n, "Speed must be in (0,1]");
    return x;
  }
  Vector vector(const YAML::Node &n, std::size_t lo, std::size_t hi) const {
    if (!n.IsSequence() || n.size() < lo || n.size() > hi)
      fail(n, "Wrong vector dimension");
    Vector r;
    for (auto x : n)
      r.push_back(number(x));
    return r;
  }
  PathVector3 xyz(const YAML::Node &n) const {
    const auto v = vector(n, 3, 3);
    return {v[0], v[1], v[2]};
  }
  std::array<double, 4> quaternion(const YAML::Node &n) const {
    const auto v = vector(n, 4, 4);
    std::array<double, 4> q{v[0], v[1], v[2], v[3]};
    if (!kinematic_detail::finite_pose({{}, q}))
      fail(n, "Expected normalized quaternion [w,x,y,z]");
    return q;
  }
  ProgramPoint point(const YAML::Node &n) const {
    keys(n, {"joints", "position", "orientation"});
    if (n["joints"]) {
      if (n["position"] || n["orientation"])
        fail(n, "Point mixes joints and pose");
      return vector(n["joints"], 1, 12);
    }
    return Pose{xyz(n["position"]), quaternion(n["orientation"])};
  }
};
class Guard final : public YAML::EventHandler {
public:
  explicit Guard(std::string f) : file_(std::move(f)) {}
  void OnDocumentStart(const YAML::Mark &m) override {
    if (documents_++)
      fail(m, "Expected exactly one YAML document");
  }
  void OnDocumentEnd() override {}
  void OnNull(const YAML::Mark &m, YAML::anchor_t) override { node(m); }
  void OnAlias(const YAML::Mark &m, YAML::anchor_t) override { fail(m, "YAML aliases are unsupported"); }
  void OnScalar(const YAML::Mark &m, const std::string &t, YAML::anchor_t, const std::string &) override {
    node(m);
    tag(m, t);
  }
  void OnSequenceStart(const YAML::Mark &m, const std::string &t, YAML::anchor_t,
                       YAML::EmitterStyle::value) override {
    start(m, t);
  }
  void OnSequenceEnd() override { --depth_; }
  void OnMapStart(const YAML::Mark &m, const std::string &t, YAML::anchor_t,
                  YAML::EmitterStyle::value) override {
    start(m, t);
  }
  void OnMapEnd() override { --depth_; }

private:
  [[noreturn]] void fail(const YAML::Mark &m, const char *s) {
    throw RobotProgramError({file_, "program", std::size_t(m.line + 1), std::size_t(m.column + 1), 0}, s);
  }
  void node(const YAML::Mark &m) {
    if (++nodes_ > 50000)
      fail(m, "YAML node budget exceeded");
  }
  void tag(const YAML::Mark &m, const std::string &t) {
    if (!t.empty() && t != "?" && t != "!")
      fail(m, "Explicit YAML tags are unsupported");
  }
  void start(const YAML::Mark &m, const std::string &t) {
    node(m);
    tag(m, t);
    if (++depth_ > 32)
      fail(m, "YAML nesting budget exceeded");
  }
  std::string file_;
  std::size_t documents_{}, nodes_{}, depth_{};
};
struct Statement {
  std::optional<ProgramInstruction> motion;
  std::string call;
  ProgramLocation location;
};
} // namespace
RobotProgramError::RobotProgramError(ProgramLocation l, const std::string &m)
    : std::runtime_error(describe(l, m)), location_(std::move(l)), detail_(m) {}
RobotProgramSource parse_robot_program_yaml(std::string_view content, const std::string &file) {
  Reader r{file};
  if (content.empty() || content.size() > 1024 * 1024 || content.find('\0') != content.npos)
    throw RobotProgramError({file, "program", 0, 0, 0}, "Empty/oversized/invalid program");
  try {
    std::istringstream input{std::string(content)};
    YAML::Parser parser(input);
    Guard guard(file);
    while (parser.HandleNextDocument(guard)) {
    }
    const auto root = YAML::Load(std::string(content));
    r.keys(root, {"schema_version", "units", "frame", "robot_profile", "robot_urdf", "start", "points",
                  "steps", "subprograms", "speed", "cycle_s", "epoch_s"});
    if (r.number(root["schema_version"]) != 1 || r.text(root["units"]) != "SI")
      r.fail(root, "Expected schema_version 1 and units SI");
    if (root["frame"] && r.text(root["frame"]) != "world")
      r.fail(root["frame"], "Only world frame is supported");
    RobotProgramSource result;
    result.filename_ = file;
    result.profile_ = r.text(root["robot_profile"]);
    result.robot_location_ = r.location(root["robot_profile"], "robot");
    result.start_location_ = r.location(root["start"]);
    if (root["robot_urdf"])
      result.urdf_ = r.text(root["robot_urdf"]);
    if (root["cycle_s"])
      result.cycle_ = r.positive(root["cycle_s"]);
    if (result.cycle_ < .0001 || result.cycle_ > 1)
      r.fail(root["cycle_s"], "Cycle must be 0.0001..1 seconds");
    if (root["epoch_s"])
      result.epoch_ = r.number(root["epoch_s"]);
    const double global = root["speed"] ? r.speed(root["speed"]) : 1;
    std::map<std::string, ProgramPoint> points;
    if (root["points"]) {
      const auto p = root["points"];
      if (!p.IsMap() || p.size() > 4096)
        r.fail(p, "Expected at most 4096 named points");
      for (auto f : p) {
        const auto name = r.text(f.first);
        if (!points.emplace(name, r.point(f.second)).second)
          r.fail(f.first, "Duplicate point: " + name);
      }
    }
    const auto target = [&](const YAML::Node &n) -> ProgramPoint {
      if (!n.IsScalar())
        return r.point(n);
      const auto name = r.text(n);
      if (!points.count(name))
        r.fail(n, "Unknown point: " + name);
      return points.at(name);
    };
    auto initial = target(root["start"]);
    if (!std::holds_alternative<Vector>(initial))
      r.fail(root["start"], "Start must specify joints");
    result.start_ = std::get<Vector>(initial);
    std::size_t statements = 0;
    const auto read = [&](const YAML::Node &sequence, const std::string &block) {
      if (!sequence.IsSequence() || sequence.size() == 0)
        r.fail(sequence, "Expected nonempty step sequence");
      std::vector<Statement> out;
      for (std::size_t i = 0; i < sequence.size(); ++i) {
        const auto n = sequence[i];
        const auto location = r.location(n, block, i + 1);
        try {
          if (++statements > 10000)
            r.fail(n, "Statement budget exceeded");
          if (!n.IsMap())
            r.fail(n, "Expected step mapping");
          const auto type = r.text(n["type"]);
          Statement step;
          step.location = location;
          if (type == "CALL") {
            r.keys(n, {"type", "program"});
            step.call = r.text(n["program"]);
          } else {
            ProgramInstruction op;
            op.location = location;
            if (type == "PTP" || type == "LIN") {
              r.keys(n, {"type", "target", "joints", "speed"});
              if (bool(n["target"]) == bool(n["joints"]))
                r.fail(n, "Specify exactly one target or joints");
              if (type == "LIN" && n["joints"])
                r.fail(n, "LIN uses target; joints shortcut is PTP only");
              op.operation = type == "PTP" ? ProgramOperation::Ptp : ProgramOperation::Line;
              op.target = n["target"] ? target(n["target"]) : ProgramPoint(r.vector(n["joints"], 1, 12));
            } else if (type == "CIRC") {
              r.keys(n, {"type", "center", "center_offset", "normal", "sweep_rad", "orientation", "speed"});
              if (bool(n["center"]) == bool(n["center_offset"]))
                r.fail(n, "Specify exactly one center or center_offset");
              op.operation = ProgramOperation::Circle;
              op.relative_center = bool(n["center_offset"]);
              op.center = r.xyz(op.relative_center ? n["center_offset"] : n["center"]);
              op.normal = r.xyz(n["normal"]);
              op.sweep_rad = r.number(n["sweep_rad"]);
              if (op.sweep_rad == 0 || std::abs(op.sweep_rad) > 2 * kinematic_detail::pi ||
                  kinematic_detail::norm(op.normal) == 0)
                r.fail(n, "Invalid CIRC sweep/normal");
              if (n["orientation"])
                op.orientation = r.quaternion(n["orientation"]);
            } else if (type == "WAIT") {
              r.keys(n, {"type", "seconds"});
              op.operation = ProgramOperation::Wait;
              op.seconds = r.positive(n["seconds"]);
              if (op.seconds > 3600)
                r.fail(n, "WAIT exceeds one hour");
            } else
              r.fail(n["type"], "Unsupported instruction: " + type);
            op.speed = global * (n["speed"] ? r.speed(n["speed"]) : 1);
            if (op.speed < .001)
              r.fail(n, "Effective speed must be at least 0.001");
            step.motion = op;
          }
          out.push_back(std::move(step));
        } catch (const RobotProgramError &e) {
          auto l = e.location();
          l.block = block;
          l.step = i + 1;
          if (l.line == 0) {
            l.line = location.line;
            l.column = location.column;
          }
          throw RobotProgramError(l, e.detail());
        }
      }
      return out;
    };
    std::map<std::string, std::vector<Statement>> definitions;
    if (root["subprograms"]) {
      auto subs = root["subprograms"];
      if (!subs.IsMap() || subs.size() > 256)
        r.fail(subs, "Expected at most 256 subprograms");
      for (auto f : subs) {
        const auto name = r.text(f.first);
        if (definitions.count(name))
          r.fail(f.first, "Duplicate subprogram: " + name);
        definitions.emplace(name, read(f.second, "subprograms." + name));
      }
    }
    const auto main = read(root["steps"], "steps");
    std::map<std::string, int> colors;
    const auto validate = [&](auto &&self, const std::vector<Statement> &steps, std::size_t depth) -> void {
      for (const auto &s : steps)
        if (!s.call.empty()) {
          if (!definitions.count(s.call))
            throw RobotProgramError(s.location, "Unknown subprogram: " + s.call);
          if (colors[s.call] == 1)
            throw RobotProgramError(s.location, "Recursive subprogram call");
          if (depth >= 16)
            throw RobotProgramError(s.location, "Subprogram depth budget exceeded");
          if (colors[s.call] == 0) {
            colors[s.call] = 1;
            self(self, definitions.at(s.call), depth + 1);
            colors[s.call] = 2;
          }
        }
    };
    for (const auto &[name, steps] : definitions) {
      colors[name] = 1;
      validate(validate, steps, 0);
      colors[name] = 2;
    }
    validate(validate, main, 0);
    const auto expand = [&](auto &&self, const std::vector<Statement> &steps, std::size_t depth) -> void {
      if (depth > 16)
        throw RobotProgramError({file, "steps", 0, 0, 0}, "Subprogram depth budget exceeded");
      for (const auto &s : steps) {
        if (s.motion) {
          if (result.steps_.size() >= 10000)
            throw RobotProgramError(s.location, "Expanded instruction budget exceeded");
          result.steps_.push_back(*s.motion);
        } else
          self(self, definitions.at(s.call), depth + 1);
      }
    };
    expand(expand, main, 0);
    return result;
  } catch (const RobotProgramError &) {
    throw;
  } catch (const YAML::Exception &e) {
    throw RobotProgramError({file, "program", e.mark.is_null() ? 0 : std::size_t(e.mark.line + 1),
                             e.mark.is_null() ? 0 : std::size_t(e.mark.column + 1), 0},
                            e.msg);
  }
}
RobotProgramSource load_robot_program_source(const std::string &file) {
  std::ifstream in(file, std::ios::binary | std::ios::ate);
  if (!in)
    throw RobotProgramError({file, "program", 0, 0, 0}, "Cannot open program");
  auto n = in.tellg();
  if (n < 0 || n > 1024 * 1024)
    throw RobotProgramError({file, "program", 0, 0, 0}, "Program exceeds 1 MiB");
  std::string s(std::size_t(n), '\0');
  in.seekg(0);
  if (!in.read(s.data(), std::streamsize(s.size())))
    throw RobotProgramError({file, "program", 0, 0, 0}, "Cannot read program");
  return parse_robot_program_yaml(s, file);
}
PreparedRobotProgram load_robot_program(const std::string &file) {
  const auto source = load_robot_program_source(file);
  const auto directory = std::filesystem::absolute(file).parent_path(),
             profile = (directory / source.robot_profile()).lexically_normal();
  const auto urdf = source.robot_urdf().empty() ? profile.parent_path() / "robot.urdf"
                                                : (directory / source.robot_urdf()).lexically_normal();
  try {
    return compile_robot_program(source, load_serial_robot_files(urdf.string(), profile.string()));
  } catch (const RobotProgramError &) {
    throw;
  } catch (const std::exception &e) {
    throw RobotProgramError(source.robot_location(), e.what());
  }
}
} // namespace motion
