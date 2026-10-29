#include "motion/program/robot_program.hpp"
#include <cassert>
#include <iostream>
using namespace motion;
namespace {
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
std::string document(const std::string &body) {
  return "0\nSECTION\n2\nENTITIES\n" + body + "0\nENDSEC\n0\nEOF\n";
}
std::string prefix =
    "schema_version: 1\nunits: SI\nrobot_profile: profile.yaml\nstart: {joints: [0.1,0.2,0.3]}\n";
const std::string line = document("0\nLINE\n8\nPATH\n10\n0\n20\n0\n11\n10\n21\n0\n");
template <class F> void rejected(F run, const std::string &part) {
  bool caught = false;
  try {
    run();
  } catch (const RobotProgramError &error) {
    caught = true;
    if (std::string(error.what()).find(part) == std::string::npos) {
      std::cerr << error.what() << " expected " << part << '\n';
      std::abort();
    }
    assert(error.location().step > 0);
  }
  assert(caught);
}
RobotProgramSource parse(const std::string &steps) {
  return parse_robot_program_yaml(prefix + steps, "dxf-test.yaml");
}
void transforms_and_cache() {
  const auto source =
      parse("subprograms: {draw: [{type: DXF, file: contour.dxf, units: mm, reference: tcp, travel: "
            "reject}]}\nsteps: [{type: CALL, program: draw}, {type: CALL, program: draw}]\n");
  int loads = 0;
  const auto p = compile_robot_program(source, xyz(), [&](const std::string &name) {
    assert(name == "contour.dxf");
    ++loads;
    return line;
  });
  assert(loads == 1 && p.reference->segment_count() == 2);
  const auto final = simulate_robot_program(p).final_state;
  assert(std::abs(final.q[0] - .12) < 1e-8 && std::abs(final.q[1] - .2) < 1e-8);
  assert(p.reference->instruction(0).drawing_location->entity_type == "LINE");
  assert(p.reference->instruction(0).location.block == "subprograms.draw");
  const auto world =
      parse("steps: [{type: DXF, file: shape.dxf, units: mm, reference: world, placement: {position: "
            "[0.1,0.2,0.3], orientation: [0.7071067811865476,0,0,0.7071067811865475]}}]");
  const auto rotated = compile_robot_program(world, xyz(), [&](const auto &) { return line; });
  const auto q = simulate_robot_program(rotated).final_state.q;
  assert(std::abs(q[0] - .1) < 1e-8 && std::abs(q[1] - .21) < 1e-8 && std::abs(q[2] - .3) < 1e-8);
}
void points_and_travel() {
  const auto drawing = document("0\nPOINT\n10\n1\n20\n0\n0\nPOINT\n10\n2\n20\n0\n");
  for (const auto &travel : {"LIN", "PTP"}) {
    const auto source =
        parse(std::string("steps: [{type: DXF, file: points.dxf, units: mm, travel: ") + travel + "}]");
    const auto p = compile_robot_program(source, xyz(), [&](const auto &) { return drawing; });
    assert(p.reference->segment_count() == 2);
    assert(std::abs(simulate_robot_program(p).final_state.q[0] - .102) < 1e-8);
  }
  rejected(
      [&] {
        compile_robot_program(parse("steps: [{type: DXF, file: points.dxf, units: mm}]"), xyz(),
                              [&](const auto &) { return drawing; });
      },
      "explicit travel");
}
void arcs_and_rejection() {
  const auto circle = document("0\nCIRCLE\n10\n0\n20\n0\n40\n10\n");
  const auto source =
      parse("steps: [{type: DXF, file: circle.dxf, units: mm, travel: LIN, path_tolerance_m: 0.00001}]");
  const auto p = compile_robot_program(source, xyz(), [&](const auto &) { return circle; });
  assert(p.reference->segment_count() == 2);
  assert(p.reference->instruction(1).operation == ProgramOperation::Circle);
  for (int i = 0; i <= 80; ++i) {
    const auto state = p.reference->sample_motion(1, p.reference->segment_duration(1) * i / 80.);
    assert(std::abs(std::hypot(state.tcp.position[0] - .1, state.tcp.position[1] - .2) - .01) < 1e-5);
    assert(std::abs(state.tcp.position[2] - .3) < 1e-8);
  }
  for (const std::string value :
       {"units: degrees", "travel: FLY", "reference: base", "path_tolerance_m: .nan",
        "path_tolerance_m: 1e-9", "placement: {joints: [0,0,0]}"})
    rejected([&] { parse("steps: [{type: DXF, file: a.dxf, " + value + "}]"); }, "");
  rejected(
      [&] {
        compile_robot_program(parse("steps: [{type: DXF, file: bad.dxf, units: mm}]"), xyz(),
                              [&](const auto &) { return document("0\nSPLINE\n"); });
      },
      "DXF SPLINE");
  const auto huge = document("0\nPOINT\n10\n1e10\n");
  rejected([&] { compile_robot_program(source, xyz(), [&](const auto &) { return huge; }); },
           "coordinate budget");
  const auto far = document("0\nPOINT\n10\n3000\n");
  rejected([&] { compile_robot_program(source, xyz(), [&](const auto &) { return far; }); }, "circle.dxf");
}
} // namespace
int main() {
  transforms_and_cache();
  points_and_travel();
  arcs_and_rejection();
  const auto program =
      load_robot_program(std::string(MOTION_SOURCE_DIR) + "/programs/examples/ur5e_dxf_demo.yaml");
  const auto report = simulate_robot_program(program);
  assert(program.reference->segment_count() == 6 && is_rest(report.final_state, 0));
  std::cout << "DXF native UR5e trace: " << report.commands
            << " commands, CAD provenance and terminal rest\n";
}
