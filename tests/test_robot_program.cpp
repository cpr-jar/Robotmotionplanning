#include "motion/robot_program.hpp"
#include <cassert>
#include <iostream>
#include <sstream>
using namespace motion;
namespace {
const std::string prefix =
    "schema_version: 1\nunits: SI\nrobot_profile: profile.yaml\nstart: {joints: [0.1, 0, 0]}\n";
ImportedSerialRobot xyz() {
  const auto t = SerialJointType::Prismatic;
  auto model = std::make_shared<SerialChainKinematics>(
      std::vector<SerialJoint>{{"x", "base", "x", t, {}, {1, 0, 0}, -2, 2, 1},
                               {"y", "x", "y", t, {}, {0, 1, 0}, -2, 2, 1},
                               {"z", "y", "tool", t, {}, {0, 0, 1}, -2, 2, 1}});
  return {"xyz",
          "base",
          "tool",
          model,
          {Vector(3, -2), Vector(3, 2), Vector(3, 1), Vector(3, 2), Vector(3, 10)},
          {.2, .5, 2, .5, 1, 5}};
}
RobotProgramSource parse(const std::string &s) {
  return parse_robot_program_yaml(s, "fixture.yaml");
}
template <class F> void rejected(F f, const std::string &fragment) {
  bool caught = false;
  try {
    f();
  } catch (const std::exception &e) {
    caught = true;
    if (std::string(e.what()).find(fragment) == std::string::npos) {
      std::cerr << e.what() << " expected " << fragment << '\n';
      std::abort();
    }
  }
  assert(caught);
}
void syntax() {
  const std::string wait = "steps: [{type: WAIT, seconds: 0.02}]\n";
  for (const auto &[content, message] : std::vector<std::pair<std::string, std::string>>{
           {prefix + wait + "units: SI\n", "Duplicate"},
           {prefix + wait + "extra: 1", "Unknown"},
           {prefix + "steps: [{type: FLY}]", "Unsupported"},
           {prefix + "steps: [{type: PTP, target: missing}]", "Unknown point"},
           {prefix + "steps: [{type: PTP, joints: [0,0,0], speed: 1.1}]", "Speed"},
           {prefix + "steps: [{type: WAIT, seconds: .nan}]", "finite"},
           {prefix + "steps: [{type: WAIT, seconds: nope}]", "finite"},
           {prefix + "steps: [{type: WAIT, seconds: -1}]", "positive"},
           {prefix + "steps: [{type: WAIT, seconds: 1, seconds: 2}]", "Duplicate"},
           {prefix + "steps: [{type: PTP, target: {position: [0,0,0], orientation: [0,0,0,0]}}]",
            "quaternion"},
           {prefix + "steps: [{type: PTP, joints: [0,0,0], target: x}]", "exactly one"},
           {prefix + "steps: [{type: LIN, joints: [0,0,0]}]", "shortcut"},
           {prefix + "steps: [{type: CIRC, center: [0,0,0], normal: [0,0,0], sweep_rad: 1}]", "normal"},
           {prefix + "steps: [{type: WAIT, seconds: !degrees 1}]", "tags"},
           {prefix + "steps: &s [{type: WAIT, seconds: 1}]\npoints: {bad: *s}", "aliases"},
           {prefix + wait + "---\nx: 1", "one YAML document"},
           {prefix + "subprograms: {a: [{type: CALL, program: b}], b: [{type: CALL, program: a}]}\nsteps: "
                     "[{type: CALL, program: a}]",
            "Recursive"},
           {prefix + "steps: [{type: CALL, program: missing}]", "Unknown subprogram"},
           {prefix + "frame: tool\n" + wait, "world"},
           {prefix + "points: {a: {joints: [0]}, a: {joints: [1]}}\n" + wait, "Duplicate point"},
           {prefix + "points: {a: {joints: [0], position: [0,0,0]}}\n" + wait, "mixes"}})
    rejected([&] { parse(content); }, message);
  rejected([&] { parse(std::string(1024 * 1024 + 1, ' ')); }, "oversized");
  rejected([&] { parse(prefix + "steps: " + std::string(40, '[') + "0" + std::string(40, ']')); }, "nesting");
  std::string huge = prefix + "subprograms:\n  a:\n";
  for (int i = 0; i < 101; ++i)
    huge += "    - {type: WAIT, seconds: 0.01}\n";
  huge += "steps:\n";
  for (int i = 0; i < 101; ++i)
    huge += "  - {type: CALL, program: a}\n";
  rejected([&] { parse(huge); }, "Expanded instruction budget");
  try {
    parse(prefix + "steps:\n  - type: WAIT\n    seconds: nope\n");
    assert(false);
  } catch (const RobotProgramError &e) {
    assert(e.location().step == 1 && e.location().line == 7 && e.location().block == "steps");
  }
}
void motions() {
  const auto source = parse(prefix + R"(
points:
  home: {joints: [0.1,0,0]}
  approach: {joints: [0.12,0.01,0]}
  work: {position: [0.13,0.02,0], orientation: [1,0,0,0]}
subprograms:
  process:
    - {type: LIN, target: work, speed: 0.75}
    - {type: CIRC, center_offset: [-0.01,0,0], normal: [0,0,1], sweep_rad: 1.5707963267948966}
    - {type: WAIT, seconds: 0.021}
steps:
  - {type: PTP, target: approach}
  - {type: CALL, program: process}
  - {type: PTP, target: home}
)");
  const auto p = compile_robot_program(source, xyz());
  const auto &program = *p.reference;
  assert(program.segment_count() == 5);
  for (int k = 0; k <= 40; ++k) {
    const auto line = program.sample_motion(1, program.segment_duration(1) * (k / 40.));
    assert(std::abs((line.joints.q[0] - .12) - (line.joints.q[1] - .01)) < 1e-7);
    const auto arc = program.sample_motion(2, program.segment_duration(2) * (k / 40.));
    assert(std::abs(std::hypot(arc.tcp.position[0] - .12, arc.tcp.position[1] - .02) - .01) < .0002);
  }
  const auto end = program.sample_segment(2, program.segment_duration(2));
  assert(std::abs(end.q[0] - .12) < 1e-7 && std::abs(end.q[1] - .03) < 1e-7);
  assert(p.schedule->segment_duration(3) >= .021 && p.schedule->segment_duration(3) < .031);
  std::ostringstream csv;
  csv << std::hex << std::scientific;
  csv.precision(5);
  csv.width(3);
  auto flags = csv.flags();
  const auto report = simulate_robot_program(p, &csv);
  assert(csv.flags() == flags && csv.precision() == 5 && csv.width() == 3);
  assert(report.commands == p.schedule->total_ticks() + 1 && report.rest_boundaries == 5 &&
         report.final_state.q == source.start());
  assert(report.peak_tcp_linear[0] <= .2 + 1e-8 && report.peak_tcp_linear[1] <= .5 + 1e-8 &&
         report.peak_tcp_linear[2] <= 2 + 1e-8);
  const auto trace = csv.str();
  assert(std::uint64_t(std::count(trace.begin(), trace.end(), '\n')) == report.commands + 1);
  assert(trace.find("timestamp_s") != trace.npos && trace.find("tcp_j5") != trace.npos);
  rejected([&] { simulate_robot_program(p, nullptr, {1}); }, "budget");
  rejected(
      [&] {
        compile_robot_program(
            parse(prefix + "steps: [{type: PTP, joints: [0.2,0,0]}, {type: PTP, joints: [3,0,0]}]"), xyz());
      },
      "step 2");
  assert(simulate_robot_program(p).final_state.q == report.final_state.q);
  rejected([&] { compile_robot_program(parse(prefix + "steps: [{type: PTP, joints: [0,0]}]"), xyz()); },
           "dimension");
  rejected(
      [&] {
        compile_robot_program(
            parse(prefix + "steps: [{type: LIN, target: {position: [8,0,0], orientation: [1,0,0,0]}}]"),
            xyz());
      },
      "step 1");
  const auto fast = compile_robot_program(parse(prefix + "steps: [{type: PTP, joints: [0.2,0.1,0]}]"), xyz());
  const auto slow =
      compile_robot_program(parse(prefix + "speed: 0.5\nsteps: [{type: PTP, joints: [0.2,0.1,0]}]"), xyz());
  assert(slow.reference->duration() >= fast.reference->duration() * 1.99);
  auto robot = xyz();
  robot.tcp_limits.max_linear_velocity = .01;
  const auto capped =
      compile_robot_program(parse(prefix + "steps: [{type: PTP, joints: [0.2,0.1,0]}]"), robot);
  assert(simulate_robot_program(capped).peak_tcp_linear[0] <= .01 + 1e-8);
  const auto small =
      compile_robot_program(parse(prefix + "cycle_s: 0.0001\nsteps: [{type: WAIT, seconds: 0.001}]"), xyz());
  assert(simulate_robot_program(small).commands >= 11);
}
} // namespace
int main() {
  syntax();
  motions();
  for (auto name : {"ur5e", "abb_irb120", "fanuc_lrmate200id"}) {
    const auto p = load_robot_program(std::string(MOTION_SOURCE_DIR) + "/programs/" + name + "_demo.yaml");
    const auto report = simulate_robot_program(p);
    assert(report.rest_boundaries == 5 && is_rest(report.final_state, 0));
    const auto &robot = p.reference->robot();
    for (std::size_t k = 0; k < p.reference->segment_count(); ++k)
      for (int sample = 0; sample <= 30; ++sample) {
        const auto point = p.reference->sample_motion(k, p.reference->segment_duration(k) * (sample / 30.));
        const double speed = p.reference->instruction(k).speed;
        for (std::size_t i = 0; i < 6; ++i) {
          assert(std::abs(point.joints.dq[i]) <= robot.joint_limits.max_velocity[i] * speed + 1e-8);
          assert(std::abs(point.joints.ddq[i]) <=
                 robot.joint_limits.max_acceleration[i] * speed * speed + 1e-8);
          assert(std::abs(point.joint_jerk[i]) <=
                 robot.joint_limits.max_jerk[i] * speed * speed * speed + 1e-8);
        }
        const std::array<Twist, 3> d{point.tcp_derivatives.velocity, point.tcp_derivatives.acceleration,
                                     point.tcp_derivatives.jerk};
        const double lc[3] = {robot.tcp_limits.max_linear_velocity, robot.tcp_limits.max_linear_acceleration,
                              robot.tcp_limits.max_linear_jerk},
                     ac[3] = {robot.tcp_limits.max_angular_velocity,
                              robot.tcp_limits.max_angular_acceleration, robot.tcp_limits.max_angular_jerk};
        double scale = speed;
        for (int o = 0; o < 3; ++o) {
          assert(std::hypot(d[o].data[0], d[o].data[1], d[o].data[2]) <= lc[o] * scale + 1e-8);
          assert(std::hypot(d[o].data[3], d[o].data[4], d[o].data[5]) <= ac[o] * scale + 1e-8);
          scale *= speed;
        }
      }
    std::cout << name << ": " << report.commands << " dispatched commands, terminal rest\n";
  }
  std::cout << "Program parsing, independent geometry, caps, budgets, source locations and replay passed\n";
}
